use std::fs::File;
use std::path::Path;
use tiff::decoder::{Decoder, DecodingResult};

pub struct DemReader {
    pub width: u32,
    pub height: u32,
    pub data: Vec<f32>,
    pub origin_x: f64,
    pub origin_y: f64,
    pub pixel_width: f64,
    pub pixel_height: f64,
}

impl DemReader {
    pub fn new<P: AsRef<Path>>(path: P) -> anyhow::Result<Self> {
        let path_ref = path.as_ref();
        if path_ref
            .extension()
            .and_then(|s| s.to_str())
            .map(|s| s.to_lowercase())
            == Some("asc".to_string())
        {
            return Self::load_asc(path_ref);
        }

        let file = File::open(path_ref)?;
        let mut decoder = Decoder::new(file)?;
        let (width, height) = decoder.dimensions()?;

        // Tags géo standards GeoTIFF
        // ModelPixelScaleTag = 33550, ModelTiepointTag = 33922

        let mut pixel_width = 1.0;
        let mut pixel_height = 1.0;
        let mut origin_x = 0.0;
        let mut origin_y = 0.0;

        if let Ok(vec) = decoder.get_tag_f64_vec(tiff::tags::Tag::Unknown(33550)) {
            if vec.len() >= 2 {
                pixel_width = vec[0];
                pixel_height = vec[1];
            }
        }

        if let Ok(vec) = decoder.get_tag_f64_vec(tiff::tags::Tag::Unknown(33922)) {
            if vec.len() >= 6 {
                origin_x = vec[3];
                origin_y = vec[4];
            }
        }

        let img = decoder.read_image()?;
        let data = match img {
            DecodingResult::F32(vec) => vec,
            DecodingResult::U16(vec) => vec.into_iter().map(|v| v as f32).collect(),
            DecodingResult::I16(vec) => vec.into_iter().map(|v| v as f32).collect(),
            DecodingResult::U8(vec) => vec.into_iter().map(|v| v as f32).collect(),
            _ => anyhow::bail!("Format TIFF non supporté pour le MNT"),
        };

        Ok(Self {
            width,
            height,
            data,
            origin_x,
            origin_y,
            pixel_width,
            pixel_height,
        })
    }

    /// Charge un MNT depuis un fichier Esri ASCII Grid (.asc)
    fn load_asc<P: AsRef<Path>>(path: P) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        let mut lines = content.lines().filter(|l| !l.trim().is_empty());

        let mut next_val = || -> anyhow::Result<f64> {
            let line = lines
                .next()
                .ok_or_else(|| anyhow::anyhow!("Fichier .asc tronque"))?;
            let mut parts = line.split_whitespace();
            parts.next(); // skip label
            let val = parts
                .next()
                .ok_or_else(|| anyhow::anyhow!("Valeur manquante"))?;
            Ok(val.parse()?)
        };

        let ncols = next_val()? as u32;
        let nrows = next_val()? as u32;
        let xllcorner = next_val()?;
        let yllcorner = next_val()?;
        let cellsize = next_val()?;
        let nodata = next_val()? as f32;

        let mut data = Vec::with_capacity((ncols * nrows) as usize);
        for line in lines {
            for val_str in line.split_whitespace() {
                let v: f32 = val_str.parse()?;
                if (v - nodata).abs() < 1e-4 {
                    data.push(f32::NAN);
                } else {
                    data.push(v);
                }
            }
        }

        // Dans un .asc, les donnees commencent en haut a gauche.
        // L'origine (xllcorner, yllcorner) est en bas a gauche.
        // Donc origin_y pour le premier pixel (en haut à gauche) est yllcorner + nrows * cellsize
        let origin_x = xllcorner;
        let origin_y = yllcorner + (nrows as f64) * cellsize;

        Ok(Self {
            width: ncols,
            height: nrows,
            data,
            origin_x,
            origin_y,
            pixel_width: cellsize,
            pixel_height: cellsize,
        })
    }

    /// Récupère l'altitude au point géo (x, y)
    pub fn get_elevation(&self, x: f64, y: f64) -> Option<f32> {
        // y_pixel = (origin_y - y) / pixel_height (car Y va vers le bas)
        let px = (x - self.origin_x) / self.pixel_width;
        let py = (self.origin_y - y) / self.pixel_height;

        if px < 0.0
            || py < 0.0
            || px >= (self.width as f64 - 1.0)
            || py >= (self.height as f64 - 1.0)
        {
            return None;
        }

        let px_i = px.floor() as u32;
        let py_i = py.floor() as u32;
        let dx = (px - px_i as f64) as f32;
        let dy = (py - py_i as f64) as f32;

        let v00 = self.data[(py_i * self.width + px_i) as usize];
        let v10 = self.data[(py_i * self.width + px_i + 1) as usize];
        let v01 = self.data[((py_i + 1) * self.width + px_i) as usize];
        let v11 = self.data[((py_i + 1) * self.width + px_i + 1) as usize];

        let val = v00 * (1.0 - dx) * (1.0 - dy)
            + v10 * dx * (1.0 - dy)
            + v01 * (1.0 - dx) * dy
            + v11 * dx * dy;

        Some(val)
    }

    /// Calcule la variance d'altitude sur une BBox (min_x, min_y, max_x, max_y)
    pub fn compute_roughness_score(&self, bbox: (f64, f64, f64, f64)) -> Option<f64> {
        let (min_x, min_y, max_x, max_y) = bbox;

        let step_x = self.pixel_width * 2.0;
        let step_y = self.pixel_height * 2.0;

        let mut elevations = Vec::new();
        let mut cx = min_x;
        while cx <= max_x {
            let mut cy = min_y;
            while cy <= max_y {
                if let Some(z) = self.get_elevation(cx, cy) {
                    elevations.push(z as f64);
                }
                cy += step_y;
            }
            cx += step_x;
        }

        if elevations.is_empty() {
            return None;
        }

        let mean = elevations.iter().sum::<f64>() / elevations.len() as f64;
        let variance =
            elevations.iter().map(|&z| (z - mean).powi(2)).sum::<f64>() / elevations.len() as f64;

        Some(variance.sqrt()) // Retourne l'écart-type en mètres
    }
}
