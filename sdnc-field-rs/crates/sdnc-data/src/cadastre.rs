//! Module Cadastral Français — Import, détection CRS & symbologie DGFiP pour **DXF, DXF-PCI et EDIGÉO**.
//!
//! Permet de charger des plans cadastraux multi-couches avec :
//! - Détection automatique de la projection (`$PCI_PROJECTION`, `RELSA:LAMB93` / `RGF93CC42..50` / heuristique)
//! - Séparation automatique par calques cadastraux dans l'ordre de rendu (Communes → Sections → Parcelles → Bâtiments → Bornes/Textes)
//! - Symbologie DGFiP / PCI Vecteur prête pour le moteur 2D Vulkan (`sdnc-gpu`)

use crate::dxf::{aci_to_rgb, DxfDocument, DxfParser};
use crate::edigeo::{EdigeoDocument, EdigeoParser};
use crate::project::{default_field_form, LayerSource, LayerStyle, ProjectLayerSpec};
use crate::vector::{Coordinate, Feature, FeatureCollection, Geometry};
use anyhow::Result;
use sdnc_geo::FrenchCrs;
use std::collections::HashMap;
use std::path::Path;

/// Interprète une chaîne de projection issue d'un en-tête DXF-PCI (`$PCI_PROJECTION`)
/// ou d'un fichier EDIGÉO `.GEO` (`RELSA:LAMB93`, `RGF93CC44`, `LAMB2E`, `EPSG:2154`...).
pub fn parse_crs_hint(raw: &str) -> Option<FrenchCrs> {
    let s = raw.trim().to_uppercase();
    if s.is_empty() {
        return None;
    }

    // 1. Code EPSG explicite (2154, 3942..3950, 27572, 4326, 32630..32632...)
    let digits: String = s.chars().filter(|c| c.is_ascii_digit()).collect();
    if let Ok(code) = digits.parse::<u32>() {
        if let Some(crs) = FrenchCrs::from_epsg(code) {
            return Some(crs);
        }
    }

    // 2. Codes texte DGFiP / PCI / IGN
    if s.contains("LAMB93") || s.contains("L93") || s.contains("LAMBERT93") || s.contains("LAMBERT-93") {
        return Some(FrenchCrs::Lambert93);
    }
    if s.contains("CC42") {
        return Some(FrenchCrs::LambertCC42);
    }
    if s.contains("CC43") {
        return Some(FrenchCrs::LambertCC43);
    }
    if s.contains("CC44") {
        return Some(FrenchCrs::LambertCC44);
    }
    if s.contains("CC45") {
        return Some(FrenchCrs::LambertCC45);
    }
    if s.contains("CC46") {
        return Some(FrenchCrs::LambertCC46);
    }
    if s.contains("CC47") {
        return Some(FrenchCrs::LambertCC47);
    }
    if s.contains("CC48") {
        return Some(FrenchCrs::LambertCC48);
    }
    if s.contains("CC49") {
        return Some(FrenchCrs::LambertCC49);
    }
    if s.contains("CC50") {
        return Some(FrenchCrs::LambertCC50);
    }
    if s.contains("L2E") || s.contains("LAMB2E") || s.contains("LAMBERT2E") {
        return Some(FrenchCrs::LambertIIEtendu);
    }
    if s.contains("WGS84") || s.contains("CRS84") {
        return FrenchCrs::from_epsg(4326);
    }
    None
}

/// Détecte la projection d'un fichier DXF / DXF-PCI via l'en-tête `$PCI_PROJECTION` puis heuristique de coordonnées.
pub fn detect_dxf_crs(doc: &DxfDocument, fallback: FrenchCrs) -> FrenchCrs {
    if let Some(ref proj) = doc.header.pci_projection {
        if let Some(crs) = parse_crs_hint(proj) {
            return crs;
        }
    }
    if let Some(ref datum) = doc.header.pci_datum {
        if let Some(crs) = parse_crs_hint(datum) {
            return crs;
        }
    }
    // Heuristique simple sur les coordonnées moyennes des premières entités
    if let Some(c) = doc.entities.iter().find_map(|e| e.coordinates.first()) {
        if let Some(crs) = guess_crs_from_xy(c.x, c.y) {
            return crs;
        }
    }
    fallback
}

/// Détecte la projection d'un lot EDIGÉO via `.GEO` (`RELSA`) / `.SCD` (`SCRC`) puis heuristique XY.
pub fn detect_edigeo_crs(doc: &EdigeoDocument, fallback: FrenchCrs) -> FrenchCrs {
    if let Some(crs) = parse_crs_hint(&doc.projection) {
        return crs;
    }
    for obj in &doc.all_objects {
        if let Some(ref g) = obj.geometry {
            if let Some((x0, y0, _, _)) = g.bbox() {
                if let Some(crs) = guess_crs_from_xy(x0, y0) {
                    return crs;
                }
            }
        }
    }
    fallback
}

fn guess_crs_from_xy(x: f64, y: f64) -> Option<FrenchCrs> {
    if (90_000.0..=1_260_000.0).contains(&x) && (6_000_000.0..=7_200_000.0).contains(&y) {
        return Some(FrenchCrs::Lambert93);
    }
    if (1_000_000.0..=2_350_000.0).contains(&x) && (1_000_000.0..=10_500_000.0).contains(&y) {
        let zone = ((y / 1_000_000.0).floor() as i32).clamp(1, 10);
        return match zone {
            1 | 2 => Some(FrenchCrs::LambertCC42),
            3 => Some(FrenchCrs::LambertCC43),
            4 => Some(FrenchCrs::LambertCC44),
            5 => Some(FrenchCrs::LambertCC45),
            6 => Some(FrenchCrs::LambertCC46),
            7 => Some(FrenchCrs::LambertCC47),
            8 => Some(FrenchCrs::LambertCC48),
            9 => Some(FrenchCrs::LambertCC49),
            10 => Some(FrenchCrs::LambertCC50),
            _ => None,
        };
    }
    None
}

/// Ordre de dessin 2D des couches cadastrales (du fond vers le premier plan).
fn cadastral_z_order(layer_name: &str) -> u8 {
    let u = layer_name.to_uppercase();
    if u.contains("COMMUNE") {
        0
    } else if u.contains("SECTION") || u.contains("FEUILLE") || u.contains("SUBDSECT") {
        1
    } else if u.contains("PARCELLE") || u.contains("PARC") || u.contains("SUBDFISC") {
        2
    } else if u.contains("HYDRO") || u.contains("EAU") || u.contains("FLUV") {
        3
    } else if u.contains("ROUTE") || u.contains("VOIE") {
        4
    } else if u.contains("BATI") || u.contains("BATIMENT") {
        5
    } else if u.contains("BORNE") || u.contains("CANV") || u.contains("LIM") || u.contains("POINT") {
        6
    } else if u.contains("TEXT") || u.contains("HABILL") || u.contains("LIEUDIT") || u.contains("NUMVOIE") {
        7
    } else {
        4
    }
}

/// Symbologie cadastrale DGFiP / PCI Vecteur en fonction du nom de calque (et couleur ACI DXF éventuelle).
pub fn cadastral_style_for_layer(layer_name: &str, aci_rgb: Option<(u8, u8, u8)>) -> LayerStyle {
    let u = layer_name.to_uppercase();

    if u.contains("PARCELLE") || u.contains("PARC") || u.contains("SUBDFISC") {
        LayerStyle {
            stroke_rgba: [255, 215, 70, 235],
            fill_rgba: [255, 215, 70, 20],
            stroke_width: 1.35,
            point_radius: 3.5,
            label_field: Some("TEX".to_string()),
        }
    } else if u.contains("BATI") && (u.contains("LEGER") || u.contains("LÉGER") || u.contains("02")) {
        LayerStyle {
            stroke_rgba: [200, 180, 130, 240],
            fill_rgba: [200, 180, 130, 95],
            stroke_width: 1.25,
            point_radius: 3.5,
            label_field: None,
        }
    } else if u.contains("BATI") || u.contains("BÂTI") {
        LayerStyle {
            stroke_rgba: [220, 125, 55, 255],
            fill_rgba: [235, 165, 80, 165],
            stroke_width: 1.5,
            point_radius: 3.5,
            label_field: None,
        }
    } else if u.contains("SECTION") || u.contains("FEUILLE") || u.contains("SUBDSECT") {
        LayerStyle {
            stroke_rgba: [255, 80, 130, 245],
            fill_rgba: [255, 80, 130, 12],
            stroke_width: 2.3,
            point_radius: 4.0,
            label_field: Some("TEX".to_string()),
        }
    } else if u.contains("COMMUNE") {
        LayerStyle {
            stroke_rgba: [215, 100, 255, 240],
            fill_rgba: [215, 100, 255, 10],
            stroke_width: 2.6,
            point_radius: 4.0,
            label_field: Some("TEX".to_string()),
        }
    } else if u.contains("HYDRO") || u.contains("EAU") || u.contains("FLUV") {
        LayerStyle {
            stroke_rgba: [65, 180, 255, 245],
            fill_rgba: [65, 180, 255, 75],
            stroke_width: 1.8,
            point_radius: 3.5,
            label_field: Some("TEX".to_string()),
        }
    } else if u.contains("ROUTE") || u.contains("VOIE") && !u.contains("NUMVOIE") {
        LayerStyle {
            stroke_rgba: [230, 235, 245, 225],
            fill_rgba: [230, 235, 245, 28],
            stroke_width: 1.9,
            point_radius: 3.5,
            label_field: Some("TEX".to_string()),
        }
    } else if u.contains("BORNE") || u.contains("CANV") || u.contains("POINT") || u.contains("SYMBLIM") {
        LayerStyle {
            stroke_rgba: [50, 240, 170, 255],
            fill_rgba: [50, 240, 170, 200],
            stroke_width: 1.6,
            point_radius: 4.5,
            label_field: Some("text".to_string()),
        }
    } else if u.contains("TEXT")
        || u.contains("HABILL")
        || u.contains("LIEU")
        || u.contains("NUMVOIE")
        || u.contains("ZONCOMMUNI")
        || u.contains("LABEL")
    {
        LayerStyle {
            stroke_rgba: [245, 245, 230, 240],
            fill_rgba: [245, 245, 230, 180],
            stroke_width: 1.2,
            point_radius: 2.5,
            label_field: Some("text".to_string()),
        }
    } else if let Some((r, g, b)) = aci_rgb {
        LayerStyle {
            stroke_rgba: [r, g, b, 240],
            fill_rgba: [r, g, b, 45],
            stroke_width: 1.5,
            point_radius: 4.0,
            label_field: Some("text".to_string()),
        }
    } else {
        LayerStyle::default()
    }
}

/// Charge un plan cadastral **DXF ou DXF-PCI** en le découpant par calques (couleurs ACI + symbologie DGFiP).
pub fn load_dxf_cadastral_layers(
    path: &Path,
    fallback_crs: FrenchCrs,
) -> Result<(FrenchCrs, Vec<(ProjectLayerSpec, FeatureCollection)>)> {
    let doc = DxfParser::parse(path)?;
    let crs = detect_dxf_crs(&doc, fallback_crs);
    let stem = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("dxf")
        .to_string();

    let layer_colors: HashMap<&str, i32> = doc
        .layers
        .iter()
        .map(|l| (l.name.as_str(), l.color))
        .collect();

    let mut layer_names: Vec<String> = doc.entities_by_layer.keys().cloned().collect();
    layer_names.sort_by(|a, b| {
        cadastral_z_order(a)
            .cmp(&cadastral_z_order(b))
            .then_with(|| a.cmp(b))
    });

    let mut out = Vec::new();
    for lname in layer_names {
        let fc = doc.layer_to_feature_collection(&lname);
        if fc.features.is_empty() {
            continue;
        }
        let aci_rgb = layer_colors.get(lname.as_str()).map(|&c| aci_to_rgb(c));
        let mut style = cadastral_style_for_layer(&lname, aci_rgb);
        // Si le calque DXF contient des entités TEXT/MTEXT, afficher leur propriété "text"
        if fc.features.iter().any(|f| f.get_string("text").is_some()) {
            style.label_field = Some("text".to_string());
        }

        let spec = ProjectLayerSpec {
            id: format!("dxf_{}_{}", stem, sanitize_id(&lname)),
            name: format!("{} · {}", stem, lname),
            source: LayerSource::DxfFile {
                rel_path: path.to_string_lossy().to_string(),
            },
            crs,
            visible: true,
            editable: false,
            opacity: 1.0,
            style,
            form_fields: default_field_form(),
        };
        out.push((spec, fc));
    }

    Ok((crs, out))
}

/// Charge un lot cadastral **EDIGÉO** (fichier `.THF` ou dossier) et le sépare par types d'objets DGFiP.
pub fn load_edigeo_cadastral_layers(
    path: &Path,
    fallback_crs: FrenchCrs,
) -> Result<(FrenchCrs, Vec<(ProjectLayerSpec, FeatureCollection)>)> {
    let doc = EdigeoParser::parse(path)?;
    let crs = detect_edigeo_crs(&doc, fallback_crs);
    let stem = path
        .file_stem()
        .or_else(|| path.file_name())
        .and_then(|s| s.to_str())
        .unwrap_or("edigeo")
        .to_string();

    let mut types: Vec<String> = doc.available_types();
    types.sort_by(|a, b| {
        cadastral_z_order(a)
            .cmp(&cadastral_z_order(b))
            .then_with(|| a.cmp(b))
    });

    let mut out = Vec::new();
    for type_key in types {
        let Some(objs) = doc.objects.get(&type_key) else {
            continue;
        };
        let mut fc = FeatureCollection::empty();
        for obj in objs {
            let Some(ref geom) = obj.geometry else {
                continue;
            };
            let mut feat = Feature::with_geometry(geom.clone());
            feat.id = Some(obj.id.clone());
            feat.set_property("id", obj.id.clone());
            feat.set_property("type", type_key.clone());
            for (k, v) in &obj.attributes {
                feat.set_property(k.as_str(), v.clone());
            }
            // Normaliser un champ d'étiquette lisible (TEX > IDU > id)
            let best_label = obj
                .attributes
                .get("TEX")
                .or_else(|| obj.attributes.get("IDU"))
                .cloned();
            if let Some(lbl) = best_label {
                feat.set_property("label", lbl);
            }
            fc.add_feature(feat);
        }
        if fc.features.is_empty() {
            continue;
        }

        let mut style = cadastral_style_for_layer(&type_key, None);
        if fc.features.iter().any(|f| f.get_string("label").is_some()) {
            style.label_field = Some("label".to_string());
        }

        let spec = ProjectLayerSpec {
            id: format!("edigeo_{}_{}", stem, sanitize_id(&type_key)),
            name: format!("EDIGÉO · {}", type_key),
            source: LayerSource::EdigeoDir {
                rel_path: path.to_string_lossy().to_string(),
            },
            crs,
            visible: true,
            editable: false,
            opacity: 1.0,
            style,
            form_fields: default_field_form(),
        };
        out.push((spec, fc));
    }

    Ok((crs, out))
}

/// Génère un îlot cadastral français de démonstration (section AB, parcelles irrégulières,
/// bâtiments durs concaves en L/U + bâtis légers, cours d'eau, voirie, textes & bornes)
/// pour tester instantanément le rendu 2D Vulkan sans fichier externe.
pub fn generate_demo_cadastral_layers(
    cx: f64,
    cy: f64,
    crs: FrenchCrs,
) -> Vec<(ProjectLayerSpec, FeatureCollection)> {
    let pt = |dx: f64, dy: f64| Coordinate::new(cx + dx, cy + dy);

    // 1. Section cadastrale AB
    let mut fc_sec = FeatureCollection::empty();
    let mut sec = Feature::with_geometry(Geometry::Polygon {
        coordinates: vec![vec![
            pt(-130.0, -90.0),
            pt(130.0, -90.0),
            pt(135.0, 95.0),
            pt(-125.0, 95.0),
            pt(-130.0, -90.0),
        ]],
    });
    sec.set_property("TEX", "SECTION AB");
    fc_sec.add_feature(sec);

    // 2. Parcelles cadastrales (6 parcelles contiguës avec numéros PCI)
    let mut fc_parc = FeatureCollection::empty();
    let parcel_defs: &[(&str, Vec<(f64, f64)>)] = &[
        ("0142", vec![(-110.0, -70.0), (-30.0, -70.0), (-25.0, 5.0), (-110.0, 10.0)]),
        ("0143", vec![(-30.0, -70.0), (45.0, -70.0), (48.0, 5.0), (-25.0, 5.0)]),
        ("0144", vec![(45.0, -70.0), (115.0, -68.0), (118.0, 8.0), (48.0, 5.0)]),
        ("0145", vec![(-110.0, 22.0), (-20.0, 20.0), (-18.0, 82.0), (-108.0, 80.0)]),
        ("0146", vec![(-20.0, 20.0), (55.0, 22.0), (52.0, 82.0), (-18.0, 82.0)]),
        ("0147", vec![(55.0, 22.0), (118.0, 24.0), (120.0, 82.0), (52.0, 82.0)]),
    ];
    for (num, corners) in parcel_defs {
        let mut ring: Vec<Coordinate> = corners.iter().map(|&(dx, dy)| pt(dx, dy)).collect();
        ring.push(ring[0]);
        let mut f = Feature::with_geometry(Geometry::Polygon {
            coordinates: vec![ring],
        });
        f.set_property("TEX", *num);
        f.set_property("IDU", format!("13059000AB{}", num));
        fc_parc.add_feature(f);
    }

    // 3. Bâtiments durs (dont polygones concaves en L et en U pour valider l'ear-clipping GPU !)
    let mut fc_bati_dur = FeatureCollection::empty();
    let bati_defs: Vec<Vec<(f64, f64)>> = vec![
        // Bâtiment en L (concave) sur parcelle 0142
        vec![
            (-92.0, -52.0),
            (-48.0, -52.0),
            (-48.0, -34.0),
            (-72.0, -34.0),
            (-72.0, -12.0),
            (-92.0, -12.0),
        ],
        // Bâtiment en U (concave) sur parcelle 0143
        vec![
            (-12.0, -54.0),
            (32.0, -54.0),
            (32.0, -18.0),
            (18.0, -18.0),
            (18.0, -40.0),
            (2.0, -40.0),
            (2.0, -18.0),
            (-12.0, -18.0),
        ],
        // Villa rectangulaire sur parcelle 0144
        vec![(62.0, -48.0), (98.0, -48.0), (98.0, -22.0), (62.0, -22.0)],
        // Bâtiment en L sur parcelle 0145
        vec![
            (-88.0, 34.0),
            (-36.0, 34.0),
            (-36.0, 50.0),
            (-64.0, 50.0),
            (-64.0, 68.0),
            (-88.0, 68.0),
        ],
        // Bâtiment sur parcelle 0146
        vec![(0.0, 36.0), (38.0, 36.0), (38.0, 62.0), (0.0, 62.0)],
    ];
    for (i, corners) in bati_defs.into_iter().enumerate() {
        let mut ring: Vec<Coordinate> = corners.iter().map(|&(dx, dy)| pt(dx, dy)).collect();
        ring.push(ring[0]);
        let mut f = Feature::with_geometry(Geometry::Polygon {
            coordinates: vec![ring],
        });
        f.set_property("DUR", "01");
        f.set_property("id", format!("BATI_DUR_{}", i + 1));
        fc_bati_dur.add_feature(f);
    }

    // 4. Bâtiments légers (abris / dépendances)
    let mut fc_bati_leg = FeatureCollection::empty();
    let mut shed = Feature::with_geometry(Geometry::Polygon {
        coordinates: vec![vec![
            pt(76.0, 44.0),
            pt(96.0, 44.0),
            pt(96.0, 58.0),
            pt(76.0, 58.0),
            pt(76.0, 44.0),
        ]],
    });
    shed.set_property("DUR", "02");
    fc_bati_leg.add_feature(shed);

    // 5. Voirie (Chemin communal traversant)
    let mut fc_voie = FeatureCollection::empty();
    let mut road = Feature::with_geometry(Geometry::LineString {
        coordinates: vec![pt(-135.0, 15.0), pt(0.0, 12.5), pt(135.0, 16.0)],
    });
    road.set_property("TEX", "Chemin des Garrigues");
    fc_voie.add_feature(road);

    // 6. Habillage / Textes (noms de voies & bornes limites)
    let mut fc_txt = FeatureCollection::empty();
    let mut lbl_voie = Feature::with_geometry(Geometry::Point {
        coordinates: pt(0.0, 13.0),
    });
    lbl_voie.set_property("text", "Chemin des Garrigues");
    fc_txt.add_feature(lbl_voie);

    let mut fc_bornes = FeatureCollection::empty();
    for (i, &(dx, dy)) in [
        (-110.0, -70.0),
        (-30.0, -70.0),
        (45.0, -70.0),
        (115.0, -68.0),
        (-25.0, 5.0),
        (48.0, 5.0),
    ]
    .iter()
    .enumerate()
    {
        let mut b = Feature::with_geometry(Geometry::Point {
            coordinates: pt(dx, dy),
        });
        b.set_property("text", format!("B{}", i + 1));
        fc_bornes.add_feature(b);
    }

    let make_spec = |id: &str, name: &str, style_key: &str| ProjectLayerSpec {
        id: id.to_string(),
        name: name.to_string(),
        source: LayerSource::DxfFile {
            rel_path: "demo_cadastre_pci.dxf".to_string(),
        },
        crs,
        visible: true,
        editable: false,
        opacity: 1.0,
        style: cadastral_style_for_layer(style_key, None),
        form_fields: default_field_form(),
    };

    vec![
        (make_spec("demo_cad_section", "Cadastre Démo · Section AB", "SECTION"), fc_sec),
        (make_spec("demo_cad_parcelles", "Cadastre Démo · Parcelles PCI", "PARCELLE"), fc_parc),
        (make_spec("demo_cad_voie", "Cadastre Démo · Voirie", "TRONROUTE"), fc_voie),
        (make_spec("demo_cad_bati_dur", "Cadastre Démo · Bâtiments Durs", "BATIMENT_DUR"), fc_bati_dur),
        (make_spec("demo_cad_bati_leg", "Cadastre Démo · Bâtiments Légers", "BATI_LEGER"), fc_bati_leg),
        (make_spec("demo_cad_bornes", "Cadastre Démo · Bornes Limites", "BORNE"), fc_bornes),
        (make_spec("demo_cad_textes", "Cadastre Démo · Habillage Textes", "HABILLAGE"), fc_txt),
    ]
}

fn sanitize_id(s: &str) -> String {
    s.chars()
        .map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_lowercase() } else { '_' })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dxf::DxfWriter;

    #[test]
    fn test_dxf_pci_roundtrip_and_cadastral_layers() {
        let dir = std::env::temp_dir().join("sdnc_test_dxf_pci");
        let _ = std::fs::create_dir_all(&dir);
        let dxf_path = dir.join("feuille_ab.dxf");

        let layers = vec![
            (
                "PARCELLE".to_string(),
                2,
                vec![(
                    vec![
                        (894500.0, 6269500.0, 0.0),
                        (894540.0, 6269500.0, 0.0),
                        (894540.0, 6269530.0, 0.0),
                        (894500.0, 6269530.0, 0.0),
                    ],
                    true,
                    None,
                )],
            ),
            (
                "BATIMENT_DUR".to_string(),
                1,
                vec![(
                    vec![
                        (894510.0, 6269510.0, 0.0),
                        (894525.0, 6269510.0, 0.0),
                        (894525.0, 6269520.0, 0.0),
                        (894510.0, 6269520.0, 0.0),
                    ],
                    true,
                    None,
                )],
            ),
            (
                "HABILLAGE_TEXT".to_string(),
                7,
                vec![(vec![(894520.0, 6269515.0, 0.0)], false, Some("0142".to_string()))],
            ),
        ];

        DxfWriter::write_dxf_pci(&dxf_path, &layers, "EPSG:2154").unwrap();

        let (crs, cad_layers) = load_dxf_cadastral_layers(&dxf_path, FrenchCrs::LambertCC44).unwrap();
        assert_eq!(crs, FrenchCrs::Lambert93);
        assert_eq!(cad_layers.len(), 3);

        // Vérifier que les polygones POLYLINE+VERTEX+SEQEND ont bien tous leurs sommets
        let parc = cad_layers
            .iter()
            .find(|(s, _)| s.name.contains("PARCELLE"))
            .expect("calque PARCELLE présent");
        assert_eq!(parc.1.features.len(), 1);
        match parc.1.features[0].geometry.as_ref().unwrap() {
            Geometry::Polygon { coordinates } => assert!(coordinates[0].len() >= 4),
            other => panic!("Attendu Polygon pour PARCELLE, reçu {:?}", other),
        }

        let txt = cad_layers
            .iter()
            .find(|(s, _)| s.name.contains("HABILLAGE"))
            .expect("calque HABILLAGE présent");
        assert!(txt
            .1
            .features
            .iter()
            .any(|f| f.get_string("text") == Some("0142")));

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_edigeo_nfz52000_official_topology_and_crs() {
        let dir = std::env::temp_dir().join("sdnc_test_edigeo_nf");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();

        std::fs::write(
            dir.join("E000AB01.THF"),
            "RTYSA03:THF\nGEOSA08:E000AB01\nLONSA10:E000AB01S1\nEOF:\n",
        )
        .unwrap();

        std::fs::write(
            dir.join("E000AB01.GEO"),
            "RTYSA03:GEO\nRELSA06:LAMB93\nEOF:\n",
        )
        .unwrap();

        let vec_content = "\
RTYSA03:ARC
RIDSA05:Arc_1
CORCC19:+894500.0;+6269500.0;
CORCC19:+894530.0;+6269500.0;
RTYSA03:ARC
RIDSA05:Arc_2
CORCC19:+894530.0;+6269500.0;
CORCC19:+894530.0;+6269525.0;
RTYSA03:ARC
RIDSA05:Arc_3
CORCC19:+894530.0;+6269525.0;
CORCC19:+894500.0;+6269525.0;
CORCC19:+894500.0;+6269500.0;
RTYSA03:PAR
RIDSA06:Face_1
RTYSA03:FEA
RIDSA07:Objet_1
SCPCP30:EDIGEO;SCD;PARCELLE_id
ATPCP24:EDIGEO;SCD;TEX_id
ATVCV04:0142
ATPCP24:EDIGEO;SCD;IDU_id
ATVCV12:13059000AB01
RTYSA03:LNK
RIDSA05:Lnk_1
FTPCP20:EDIGEO;VEC;Face_1
FTPCP19:EDIGEO;VEC;Arc_1
FTPCP19:EDIGEO;VEC;Arc_2
FTPCP19:EDIGEO;VEC;Arc_3
RTYSA03:LNK
RIDSA05:Lnk_2
FTPCP21:EDIGEO;VEC;Objet_1
FTPCP20:EDIGEO;VEC;Face_1
EOF:
";
        std::fs::write(dir.join("E000AB01S1.VEC"), vec_content).unwrap();

        let (crs, layers) = load_edigeo_cadastral_layers(&dir, FrenchCrs::LambertCC44).unwrap();
        assert_eq!(crs, FrenchCrs::Lambert93);
        assert_eq!(layers.len(), 1);
        let (spec, fc) = &layers[0];
        assert!(spec.name.contains("Parcelle"));
        assert_eq!(fc.features.len(), 1);
        assert_eq!(fc.features[0].get_string("TEX"), Some("0142"));
        match fc.features[0].geometry.as_ref().unwrap() {
            Geometry::Polygon { coordinates } => assert!(coordinates[0].len() >= 5),
            other => panic!("Attendu Polygon reconstruit par chaînage d'arcs, reçu {:?}", other),
        }

        let _ = std::fs::remove_dir_all(&dir);
    }
}
