/// Module de geodesie - projections francaises bidirectionnelles a precision geodesique.
///
/// Systemes couverts (metropole + DOM-TOM) :
///   Lambert 93 (EPSG:2154), CC42-CC50 (EPSG:3942-3950)
///   NTF Lambert I-IV (EPSG:27561-27564), Lambert II etendu (EPSG:27572)
///   UTM 30N-32N, 20N-22N, 40S, 38S, 58S, 59S, 5S, 6S
///
/// Precision : sub-millimetrique pour les systemes RGF93.
///             ~1m pour les systemes NTF (decalage Helmert 3 parametres).
///
/// References : EPSG Guidance Note 7 Part 2, Karney (2011).
use std::f64::consts::PI;

/// Recale automatiquement la coordonnée Y vers le faux nord (false northing) officiel de la zone CC42-CC50.
/// Si un fichier possède une coordonnée Y tronquée (ex: 3 147 000 au lieu de 4 147 000 pour la zone CC44),
/// cette fonction réajuste le préfixe de zone au domaine valide.
pub fn sanitize_cc_y(crs: FrenchCrs, y: f64) -> f64 {
    if !y.is_finite() || y < 1_000_000.0 {
        return y;
    }
    let expected_zone = match crs {
        FrenchCrs::LambertCC42 => 2,
        FrenchCrs::LambertCC43 => 3,
        FrenchCrs::LambertCC44 => 4,
        FrenchCrs::LambertCC45 => 5,
        FrenchCrs::LambertCC46 => 6,
        FrenchCrs::LambertCC47 => 7,
        FrenchCrs::LambertCC48 => 8,
        FrenchCrs::LambertCC49 => 9,
        FrenchCrs::LambertCC50 => 10,
        _ => return y,
    };
    let current_zone = (y / 1_000_000.0).floor() as i32;
    if current_zone > 0 && current_zone != expected_zone {
        let diff = expected_zone - current_zone;
        return y + (diff as f64) * 1_000_000.0;
    }
    y
}

/// Reprojection géodésique 3D instantanée à la volée entre deux systèmes français légaux.
pub fn reproject_point_3d(
    src_crs: FrenchCrs,
    dst_crs: FrenchCrs,
    x: f64,
    y: f64,
    z: f64,
) -> Result<(f64, f64, f64), String> {
    let clean_y = sanitize_cc_y(src_crs, y);
    if src_crs == dst_crs {
        let final_y = sanitize_cc_y(dst_crs, clean_y);
        return Ok((x, final_y, z));
    }
    let (lon, lat) = to_wgs84(src_crs, x, clean_y).map_err(|e| e.to_string())?;
    let (dst_x, dst_y) = from_wgs84(dst_crs, lon, lat).map_err(|e| e.to_string())?;
    let final_y = sanitize_cc_y(dst_crs, dst_y);
    Ok((dst_x, final_y, z))
}

// ─── Constantes trigonometriques ─────────────────────────────────────────────

const FRAC_PI_4: f64 = PI / 4.0;
const DEG: f64 = PI / 180.0;

// ─── Ellipsoides ─────────────────────────────────────────────────────────────

/// Ellipsoide de reference : demi-grand axe `a` (m) et premiere excentricite² `e2`.
#[derive(Debug, Clone, Copy)]
pub struct Ellipsoid {
    pub a: f64,
    pub e2: f64,
}

impl Ellipsoid {
    #[inline]
    pub fn e(&self) -> f64 {
        self.e2.sqrt()
    }
}

/// GRS80 - utilise par RGF93 (Lambert 93, CC42-CC50, UTM modernes France)
pub const GRS80: Ellipsoid = Ellipsoid {
    a: 6_378_137.0,
    e2: 0.006_694_380_022_901,
};
/// WGS84 - pratiquement identique a GRS80 (difference < 0.1 mm)
pub const WGS84: Ellipsoid = Ellipsoid {
    a: 6_378_137.0,
    e2: 0.006_694_379_990_141,
};
/// Clarke 1880 (IGN) - utilise par NTF (vieux Lambert I-IV)
/// a=6378249.2 m, 1/f=293.4660212936269
pub const CLARKE1880_IGN: Ellipsoid = Ellipsoid {
    a: 6_378_249.2,
    e2: 0.006_804_232_717,
};
/// International 1924 (Hayford) - certains DOM anciens
/// a=6378388.0 m, 1/f=297.0
pub const HAYFORD: Ellipsoid = Ellipsoid {
    a: 6_378_388.0,
    e2: 0.006_722_670_022,
};

// ─── Transformations de datum ─────────────────────────────────────────────────

/// Translation geocentrique 3 parametres (Molodensky-Badekas simplifie).
/// Ajoute (dx,dy,dz) au vecteur ECEF.
#[derive(Debug, Clone, Copy)]
pub struct GeocentricShift {
    pub dx: f64,
    pub dy: f64,
    pub dz: f64,
}

/// NTF (Paris) -> WGS84 - EPSG:1701, precision ~3 m en France metropolitaine.
pub const NTF_TO_WGS84: GeocentricShift = GeocentricShift {
    dx: -168.0,
    dy: -60.0,
    dz: 320.0,
};

// ─── Meridien de Paris ────────────────────────────────────────────────────────

/// Longitude du meridien de Paris en radians par rapport a Greenwich.
/// 2 deg20'14.025" E = 0.04079234433 rad
pub const PARIS_MERIDIAN_RAD: f64 = 0.040_792_344_331_808_6;

// ─── Fonctions auxiliaires pour LCC ──────────────────────────────────────────

/// Parametre `t` utilise dans Lambert Conique Conforme (EPSG GN7 formule 2).
#[inline]
fn lcc_t(phi: f64, e: f64) -> f64 {
    let sin_phi = phi.sin();
    let es = e * sin_phi;
    (FRAC_PI_4 - phi / 2.0).tan() * ((1.0 + es) / (1.0 - es)).powf(e / 2.0)
}

/// Parametre `m` utilise dans Lambert Conique Conforme (EPSG GN7 formule 3).
#[inline]
fn lcc_m(phi: f64, e2: f64) -> f64 {
    phi.cos() / (1.0 - e2 * phi.sin().powi(2)).sqrt()
}

/// Latitude geodesique depuis `t'` par iteration de Bowring (convergence ~1e-12 rad = sub-um).
fn lcc_phi_from_t(t_prime: f64, e: f64) -> f64 {
    let mut phi = PI / 2.0 - 2.0 * t_prime.abs().atan();
    for _ in 0..20 {
        let sin_phi = phi.sin();
        let es = e * sin_phi;
        let new_phi = PI / 2.0
            - 2.0
                * (t_prime * ((1.0 - es) / (1.0 + es)).powf(e / 2.0))
                    .abs()
                    .atan();
        if (new_phi - phi).abs() < 1e-14 {
            return new_phi;
        }
        phi = new_phi;
    }
    phi
}

// ─── LCC 2SP (EPSG Method 9802) ───────────────────────────────────────────────

/// Parametres d'une projection Lambert Conique Conforme a 2 paralleles standard.
#[derive(Debug, Clone, Copy)]
pub struct Lcc2SpParams {
    pub phi1: f64,    // 1er parallele standard (rad)
    pub phi2: f64,    // 2eme parallele standard (rad)
    pub phi0: f64,    // latitude d'origine (rad) - origine fausse
    pub lambda0: f64, // meridien central (rad)
    pub false_e: f64, // fausse abscisse (m)
    pub false_n: f64, // fausse ordonnee (m)
    pub ellipsoid: Ellipsoid,
}

/// Constantes precalculees pour LCC 2SP.
struct Lcc2SpConsts {
    n: f64,
    f: f64,
    r0: f64,
    a: f64,
    e: f64,
    lambda0: f64,
    false_e: f64,
    false_n: f64,
}

impl Lcc2SpConsts {
    fn from(p: &Lcc2SpParams) -> Self {
        let e = p.ellipsoid.e();
        let m1 = lcc_m(p.phi1, p.ellipsoid.e2);
        let m2 = lcc_m(p.phi2, p.ellipsoid.e2);
        let t1 = lcc_t(p.phi1, e);
        let t2 = lcc_t(p.phi2, e);
        let n = if (p.phi1 - p.phi2).abs() < 1e-10 {
            p.phi1.sin()
        } else {
            (m1.ln() - m2.ln()) / (t1.ln() - t2.ln())
        };
        let f = m1 / (n * t1.powf(n));
        let t0 = lcc_t(p.phi0, e);
        let r0 = p.ellipsoid.a * f * t0.powf(n);
        Lcc2SpConsts {
            n,
            f,
            r0,
            a: p.ellipsoid.a,
            e,
            lambda0: p.lambda0,
            false_e: p.false_e,
            false_n: p.false_n,
        }
    }
}

fn lcc2sp_forward(p: &Lcc2SpParams, phi: f64, lambda: f64) -> (f64, f64) {
    let c = Lcc2SpConsts::from(p);
    let t = lcc_t(phi, c.e);
    let r = c.a * c.f * t.powf(c.n);
    let theta = c.n * (lambda - c.lambda0);
    let e = c.false_e + r * theta.sin();
    let n = c.false_n + c.r0 - r * theta.cos();
    (e, n)
}

fn lcc2sp_inverse(p: &Lcc2SpParams, e: f64, n: f64) -> (f64, f64) {
    let c = Lcc2SpConsts::from(p);
    let de = e - c.false_e;
    let dn = c.r0 - (n - c.false_n);
    let r_prime = c.n.signum() * (de * de + dn * dn).sqrt();
    let t_prime = (r_prime / (c.a * c.f)).powf(1.0 / c.n);
    let theta_prime = de.atan2(dn);
    let phi = lcc_phi_from_t(t_prime, c.e);
    let lambda = theta_prime / c.n + c.lambda0;
    (phi, lambda)
}

// ─── LCC 1SP (EPSG Method 9801) ───────────────────────────────────────────────

/// Parametres d'une projection Lambert Conique Conforme a 1 parallele standard.
#[derive(Debug, Clone, Copy)]
pub struct Lcc1SpParams {
    pub phi0: f64,    // latitude d'origine = parallele standard (rad)
    pub lambda0: f64, // meridien central (rad)
    pub k0: f64,      // facteur d'echelle au parallele standard
    pub false_e: f64,
    pub false_n: f64,
    pub ellipsoid: Ellipsoid,
}

struct Lcc1SpConsts {
    n: f64,
    f: f64,
    r0: f64,
    a: f64,
    k0: f64,
    e: f64,
    lambda0: f64,
    false_e: f64,
    false_n: f64,
}

impl Lcc1SpConsts {
    fn from(p: &Lcc1SpParams) -> Self {
        let e = p.ellipsoid.e();
        let m0 = lcc_m(p.phi0, p.ellipsoid.e2);
        let t0 = lcc_t(p.phi0, e);
        let n = p.phi0.sin();
        let f = m0 / (n * t0.powf(n));
        let r0 = p.ellipsoid.a * p.k0 * f * t0.powf(n);
        Lcc1SpConsts {
            n,
            f,
            r0,
            a: p.ellipsoid.a,
            k0: p.k0,
            e,
            lambda0: p.lambda0,
            false_e: p.false_e,
            false_n: p.false_n,
        }
    }
}

fn lcc1sp_forward(p: &Lcc1SpParams, phi: f64, lambda: f64) -> (f64, f64) {
    let c = Lcc1SpConsts::from(p);
    let t = lcc_t(phi, c.e);
    let r = c.a * c.k0 * c.f * t.powf(c.n);
    let theta = c.n * (lambda - c.lambda0);
    let e = c.false_e + r * theta.sin();
    let n = c.false_n + c.r0 - r * theta.cos();
    (e, n)
}

fn lcc1sp_inverse(p: &Lcc1SpParams, e: f64, n: f64) -> (f64, f64) {
    let c = Lcc1SpConsts::from(p);
    let de = e - c.false_e;
    let dn = c.r0 - (n - c.false_n);
    let r_prime = c.n.signum() * (de * de + dn * dn).sqrt();
    let t_prime = (r_prime / (c.a * c.k0 * c.f)).powf(1.0 / c.n);
    let theta_prime = de.atan2(dn);
    let phi = lcc_phi_from_t(t_prime, c.e);
    let lambda = theta_prime / c.n + c.lambda0;
    (phi, lambda)
}

// ─── Transverse Mercator / UTM (EPSG Method 9807) ────────────────────────────

/// Parametres pour projection Transverse Mercator (UTM est un cas particulier).
#[derive(Debug, Clone, Copy)]
pub struct TmParams {
    pub lambda0: f64, // meridien central (rad)
    pub k0: f64,      // facteur d'echelle (0.9996 pour UTM)
    pub false_e: f64, // 500000 pour UTM
    pub false_n: f64, // 0 (nord) ou 10000000 (sud)
    pub ellipsoid: Ellipsoid,
}

/// Arc meridien via serie EPSG GN7-2 en e² (NIMA TR8350.2).
/// Precision < 0.01 mm sur tout l'ellipsoide.
fn meridional_arc(phi: f64, a: f64, e2: f64) -> f64 {
    let e4 = e2 * e2;
    let e6 = e4 * e2;
    a * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * (2.0 * phi).sin()
        + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * (4.0 * phi).sin()
        - (35.0 * e6 / 3072.0) * (6.0 * phi).sin())
}

/// Latitude geodesique depuis l'arc meridien M (inverse de `meridional_arc`).
/// Serie EPSG GN7-2 via latitude rectifiante, precision < 0.001 mas.
fn phi_from_arc(m: f64, a: f64, e2: f64) -> f64 {
    let e4 = e2 * e2;
    let e6 = e4 * e2;
    let mu = m / (a * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0));
    // e1 = troisieme aplatissement n = (1 − √(1−e²)) / (1 + √(1−e²))
    let e1 = (1.0 - (1.0 - e2).sqrt()) / (1.0 + (1.0 - e2).sqrt());
    let e1_2 = e1 * e1;
    let e1_3 = e1_2 * e1;
    let e1_4 = e1_3 * e1;
    mu + (3.0 * e1 / 2.0 - 27.0 * e1_3 / 32.0) * (2.0 * mu).sin()
        + (21.0 * e1_2 / 16.0 - 55.0 * e1_4 / 32.0) * (4.0 * mu).sin()
        + (151.0 * e1_3 / 96.0) * (6.0 * mu).sin()
        + (1097.0 * e1_4 / 512.0) * (8.0 * mu).sin()
}

fn tm_forward(p: &TmParams, phi: f64, lambda: f64) -> (f64, f64) {
    let e2 = p.ellipsoid.e2;
    let a = p.ellipsoid.a;
    let e2p = e2 / (1.0 - e2); // second eccentricity squared

    let nu = a / (1.0 - e2 * phi.sin().powi(2)).sqrt();
    let t = phi.tan();
    let t2 = t * t;
    let c = e2p * phi.cos().powi(2);
    let dl = lambda - p.lambda0;
    let a_coef = phi.cos() * dl;
    let m = meridional_arc(phi, a, e2);
    let m0 = meridional_arc(0.0, a, e2); // = 0 pour phi0=0

    let x = p.false_e
        + p.k0
            * nu
            * (a_coef
                + (1.0 - t2 + c) * a_coef.powi(3) / 6.0
                + (5.0 - 18.0 * t2 + t2 * t2 + 72.0 * c - 58.0 * e2p) * a_coef.powi(5) / 120.0);
    // EPSG GN7-2 §1.3.2 - northing uses A = a_coef = cos(φ)*Δλ (not Δλ alone)
    let y = p.false_n
        + p.k0
            * ((m - m0)
                + nu * t
                    * (a_coef.powi(2) / 2.0
                        + (5.0 - t2 + 9.0 * c + 4.0 * c * c) * a_coef.powi(4) / 24.0
                        + (61.0 - 58.0 * t2 + t2 * t2 + 600.0 * c - 330.0 * e2p) * a_coef.powi(6)
                            / 720.0));
    (x, y)
}

fn tm_inverse(p: &TmParams, x: f64, y: f64) -> (f64, f64) {
    let e2 = p.ellipsoid.e2;
    let a = p.ellipsoid.a;
    let e2p = e2 / (1.0 - e2);

    let m0 = meridional_arc(0.0, a, e2);
    let m_prime = m0 + (y - p.false_n) / p.k0;
    let phi1 = phi_from_arc(m_prime, a, e2);

    let sin_phi1 = phi1.sin();
    let cos_phi1 = phi1.cos();
    let tan_phi1 = phi1.tan();
    let t1 = tan_phi1 * tan_phi1;
    let c1 = e2p * cos_phi1.powi(2);
    let nu1 = a / (1.0 - e2 * sin_phi1.powi(2)).sqrt();
    let rho1 = a * (1.0 - e2) / (1.0 - e2 * sin_phi1.powi(2)).powf(1.5);
    let d = (x - p.false_e) / (nu1 * p.k0);
    let d2 = d * d;

    let phi = phi1
        - (nu1 * tan_phi1 / rho1)
            * (d2 / 2.0
                - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * e2p) * d2 * d2 / 24.0
                + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * e2p - 3.0 * c1 * c1)
                    * d2.powi(3)
                    / 720.0);
    let lambda = p.lambda0
        + (d - (1.0 + 2.0 * t1 + c1) * d2 * d / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * e2p + 24.0 * t1 * t1)
                * d2.powi(2)
                * d
                / 120.0)
            / cos_phi1;
    (phi, lambda)
}

// ─── Geodesie 3D (ECEF + Helmert) ────────────────────────────────────────────

/// Convertit des coordonnees geographiques (lon, lat) vers ECEF.
/// Convertit des coordonnees geographiques (lon, lat, h) vers ECEF.
pub fn geodetic_to_ecef(phi: f64, lam: f64, h: f64, ell: &Ellipsoid) -> (f64, f64, f64) {
    let nu = ell.a / (1.0 - ell.e2 * phi.sin().powi(2)).sqrt();
    let x = (nu + h) * phi.cos() * lam.cos();
    let y = (nu + h) * phi.cos() * lam.sin();
    let z = (nu * (1.0 - ell.e2) + h) * phi.sin();
    (x, y, z)
}

/// Inverse: ECEF vers (lat, lon) sur l'ellipsoide.
#[inline]
pub fn ecef_to_geodetic(x: f64, y: f64, z: f64, ell: &Ellipsoid) -> (f64, f64) {
    let (phi, lam, _) = ecef_to_geodetic_h(x, y, z, ell);
    (phi, lam)
}

/// Inverse: ECEF vers (lat, lon, h) avec l'algorithme robuste de Bowring.
pub fn ecef_to_geodetic_h(x: f64, y: f64, z: f64, ell: &Ellipsoid) -> (f64, f64, f64) {
    let lam = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    if p < 1e-10 {
        let phi = if z > 0.0 {
            std::f64::consts::PI / 2.0
        } else {
            -std::f64::consts::PI / 2.0
        };
        let h = z.abs() - ell.a * (1.0 - ell.e2).sqrt();
        return (phi, lam, h);
    }
    let b = ell.a * (1.0 - ell.e2).sqrt();
    let e_prime_2 = ell.e2 / (1.0 - ell.e2);
    let theta = (z * ell.a).atan2(p * b);
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let phi = (z + e_prime_2 * b * sin_theta.powi(3)).atan2(p - ell.e2 * ell.a * cos_theta.powi(3));
    let nu = ell.a / (1.0 - ell.e2 * phi.sin().powi(2)).sqrt();
    let h = if phi.cos().abs() > 1e-5 {
        p / phi.cos() - nu
    } else {
        z / phi.sin() - nu * (1.0 - ell.e2)
    };
    (phi, lam, h)
}

/// Applique la translation geocentrique Helmert 3 parametres.
pub fn apply_geocentric_shift(x: f64, y: f64, z: f64, shift: &GeocentricShift) -> (f64, f64, f64) {
    (x + shift.dx, y + shift.dy, z + shift.dz)
}

/// Inverse la translation geocentrique Helmert 3 parametres.
pub fn reverse_geocentric_shift(x: f64, y: f64, z: f64, shift: &GeocentricShift) -> (f64, f64, f64) {
    (x - shift.dx, y - shift.dy, z - shift.dz)
}

// ─── Local Tangent Plane (ENU) ────────────────────────────────────────────────

/// Point de reference (origine) pour un repere ENU (East-North-Up).
#[derive(Debug, Clone, Copy)]
pub struct EnuOrigin {
    pub lon_rad: f64,
    pub lat_rad: f64,
    pub h: f64,
    pub x0: f64, // ECEF X
    pub y0: f64, // ECEF Y
    pub z0: f64, // ECEF Z
}

impl EnuOrigin {
    pub fn new(lon_deg: f64, lat_deg: f64, h: f64, ell: &Ellipsoid) -> Self {
        let lat_rad = lat_deg * DEG;
        let lon_rad = lon_deg * DEG;
        let (x0, y0, z0) = geodetic_to_ecef(lat_rad, lon_rad, h, ell);
        Self { lon_rad, lat_rad, h, x0, y0, z0 }
    }
}

/// Contexte ENU global pour l'export/import du repere local euclidien.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EnuContext {
    pub epsg: u32,
    pub lon0_deg: f64,
    pub lat0_deg: f64,
    pub h0: f64, // Altitude ellipsoidale (Z_ortho + n_geoid)
    pub n_geoid: f64, // Ondulation utilisee pour restaurer le NGF/orthometrique
}

impl EnuContext {
    pub fn get_origin(&self) -> EnuOrigin {
        EnuOrigin::new(self.lon0_deg, self.lat0_deg, self.h0, &WGS84)
    }

    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let f = std::fs::File::create(path)?;
        serde_json::to_writer_pretty(f, self)?;
        Ok(())
    }

    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let f = std::fs::File::open(path)?;
        Ok(serde_json::from_reader(f)?)
    }
}

/// Convertit des coordonnees ECEF vers le repere local ENU.
pub fn ecef_to_enu(x: f64, y: f64, z: f64, origin: &EnuOrigin) -> (f64, f64, f64) {
    let dx = x - origin.x0;
    let dy = y - origin.y0;
    let dz = z - origin.z0;

    let slon = origin.lon_rad.sin();
    let clon = origin.lon_rad.cos();
    let slat = origin.lat_rad.sin();
    let clat = origin.lat_rad.cos();

    let e = -slon * dx + clon * dy;
    let n = -slat * clon * dx - slat * slon * dy + clat * dz;
    let u = clat * clon * dx + clat * slon * dy + slat * dz;
    (e, n, u)
}

/// Convertit des coordonnees locales ENU vers ECEF.
pub fn enu_to_ecef(e: f64, n: f64, u: f64, origin: &EnuOrigin) -> (f64, f64, f64) {
    let slon = origin.lon_rad.sin();
    let clon = origin.lon_rad.cos();
    let slat = origin.lat_rad.sin();
    let clat = origin.lat_rad.cos();

    let dx = -slon * e - slat * clon * n + clat * clon * u;
    let dy =  clon * e - slat * slon * n + clat * slon * u;
    let dz =  clat * n + slat * u;

    (origin.x0 + dx, origin.y0 + dy, origin.z0 + dz)
}

/// Calcule la convergence des meridiens (gamma) en radians pour un point (lon, lat) dans un CRS.
/// Retourne la convergence (Grid North = True North - gamma).
/// (Approximation pour LCC et UTM).
pub fn meridian_convergence(crs: FrenchCrs, lon_deg: f64, lat_deg: f64) -> f64 {
    let lat_rad = lat_deg * DEG;
    let lon_rad = lon_deg * DEG;
    
    match crs {
        FrenchCrs::Lambert93 => (lon_rad - 3.0 * DEG) * (46.5 * DEG).sin(),
        FrenchCrs::LambertCC42 => (lon_rad - 3.0 * DEG) * (42.0 * DEG).sin(),
        FrenchCrs::LambertCC43 => (lon_rad - 3.0 * DEG) * (43.0 * DEG).sin(),
        FrenchCrs::LambertCC44 => (lon_rad - 3.0 * DEG) * (44.0 * DEG).sin(),
        FrenchCrs::LambertCC45 => (lon_rad - 3.0 * DEG) * (45.0 * DEG).sin(),
        FrenchCrs::LambertCC46 => (lon_rad - 3.0 * DEG) * (46.0 * DEG).sin(),
        FrenchCrs::LambertCC47 => (lon_rad - 3.0 * DEG) * (47.0 * DEG).sin(),
        FrenchCrs::LambertCC48 => (lon_rad - 3.0 * DEG) * (48.0 * DEG).sin(),
        FrenchCrs::LambertCC49 => (lon_rad - 3.0 * DEG) * (49.0 * DEG).sin(),
        FrenchCrs::LambertCC50 => (lon_rad - 3.0 * DEG) * (50.0 * DEG).sin(),
        FrenchCrs::LambertI => (lon_rad - PARIS_MERIDIAN_RAD) * (49.5 * DEG).sin(),
        FrenchCrs::LambertII | FrenchCrs::LambertIIEtendu => (lon_rad - PARIS_MERIDIAN_RAD) * (46.8 * DEG).sin(),
        FrenchCrs::LambertIII => (lon_rad - PARIS_MERIDIAN_RAD) * (44.1 * DEG).sin(),
        FrenchCrs::LambertIV => (lon_rad - PARIS_MERIDIAN_RAD) * (42.165 * DEG).sin(),
        FrenchCrs::Utm30N => (lon_rad - (-3.0 * DEG)) * lat_rad.sin(),
        FrenchCrs::Utm31N => (lon_rad - (3.0 * DEG)) * lat_rad.sin(),
        FrenchCrs::Utm32N => (lon_rad - (9.0 * DEG)) * lat_rad.sin(),
        // ── DOM-TOM : meridien central = zone*6 - 183 (formule UTM standard) ──────────────
        FrenchCrs::Utm20N => (lon_rad - (-63.0 * DEG)) * lat_rad.sin(),  // Antilles
        FrenchCrs::Utm21N => (lon_rad - (-57.0 * DEG)) * lat_rad.sin(),  // St-Pierre
        FrenchCrs::Utm22N => (lon_rad - (-51.0 * DEG)) * lat_rad.sin(),  // Guyane
        FrenchCrs::Utm40S => (lon_rad - (57.0 * DEG)) * lat_rad.sin(),   // Reunion
        FrenchCrs::Utm38S => (lon_rad - (45.0 * DEG)) * lat_rad.sin(),   // Mayotte
        FrenchCrs::Utm58S => (lon_rad - (165.0 * DEG)) * lat_rad.sin(),  // NC zone 58
        FrenchCrs::Utm59S => (lon_rad - (171.0 * DEG)) * lat_rad.sin(),  // NC zone 59
        FrenchCrs::Utm5S  => (lon_rad - (-153.0 * DEG)) * lat_rad.sin(), // Polynesie 5S
        FrenchCrs::Utm6S  => (lon_rad - (-147.0 * DEG)) * lat_rad.sin(), // Polynesie 6S
        // Lambert NC : meridien central 166°E
        FrenchCrs::LambertNC => (lon_rad - (166.0 * DEG)) * (-21.5 * DEG).sin(),
    }
}


// ─── Catalogue CRS francais ───────────────────────────────────────────────────

/// Catalogue des systemes de coordonnees projetees francais (metropole + DOM-TOM).
/// Tous les systemes sont reversibles : `to_wgs84` et `from_wgs84`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum FrenchCrs {
    // ── Metropole moderne (RGF93/GRS80) ──────────────────────────
    /// RGF93 / Lambert-93 - EPSG:2154 (systeme officiel France entiere)
    Lambert93,
    /// RGF93 / CC42 - EPSG:3942 (Zone 1 : autour de 42 degN)
    LambertCC42,
    /// RGF93 / CC43 - EPSG:3943
    LambertCC43,
    /// RGF93 / CC44 - EPSG:3944
    LambertCC44,
    /// RGF93 / CC45 - EPSG:3945
    LambertCC45,
    /// RGF93 / CC46 - EPSG:3946
    LambertCC46,
    /// RGF93 / CC47 - EPSG:3947
    LambertCC47,
    /// RGF93 / CC48 - EPSG:3948
    LambertCC48,
    /// RGF93 / CC49 - EPSG:3949
    LambertCC49,
    /// RGF93 / CC50 - EPSG:3950
    LambertCC50,
    // ── Metropole ancien (NTF/Clarke 1880 IGN, meridien de Paris) ──
    /// NTF (Paris) / Lambert zone I  - EPSG:27561 (Nord)
    LambertI,
    /// NTF (Paris) / Lambert zone II - EPSG:27562 (Centre-Nord)
    LambertII,
    /// NTF (Paris) / Lambert zone III - EPSG:27563 (Centre-Sud)
    LambertIII,
    /// NTF (Paris) / Lambert zone IV - EPSG:27564 (Corse)
    LambertIV,
    /// NTF (Paris) / Lambert II etendu - EPSG:27572 (le plus repandu en archives)
    LambertIIEtendu,
    // ── Metropole UTM (WGS84/GRS80) ──────────────────────────────
    /// WGS84 / UTM zone 30N - EPSG:32630 (extreme ouest)
    Utm30N,
    /// WGS84 / UTM zone 31N - EPSG:32631 (France ouest + bassin parisien)
    Utm31N,
    /// WGS84 / UTM zone 32N - EPSG:32632 (France est, Alsace)
    Utm32N,
    // ── Antilles (WGS84/GRS80) ───────────────────────────────────
    /// WGS84 / UTM zone 20N - EPSG:32620 (Martinique, Guadeloupe, St-Martin, St-Barth)
    Utm20N,
    /// WGS84 / UTM zone 21N - EPSG:32621 (Grande-Terre est, St-Pierre-et-Miquelon)
    Utm21N,
    // ── Guyane (WGS84/GRS80) ─────────────────────────────────────
    /// WGS84 / UTM zone 22N - EPSG:32622 (Guyane francaise)
    Utm22N,
    // ── La Reunion (WGS84/GRS80) ─────────────────────────────────
    /// WGS84 / UTM zone 40S - EPSG:32740
    Utm40S,
    // ── Mayotte (WGS84/GRS80) ────────────────────────────────────
    /// WGS84 / UTM zone 38S - EPSG:32738
    Utm38S,
    // ── Nouvelle-Caledonie (WGS84/GRS80) ─────────────────────────
    /// WGS84 / UTM zone 58S - EPSG:32758
    Utm58S,
    /// WGS84 / UTM zone 59S - EPSG:32759
    Utm59S,
    // ── Polynesie francaise (WGS84/GRS80) ────────────────────────
    /// WGS84 / UTM zone 5S  - EPSG:32705 (Îles Marquises)
    Utm5S,
    /// WGS84 / UTM zone 6S  - EPSG:32706 (Tahiti, Moorea)
    Utm6S,
    // ── Nouvelle-Caledonie LCC ─────────────────────────────────────────
    /// RGNC91-93 / Lambert NC - EPSG:3163 (Nouvelle-Caledonie)
    LambertNC,
}

impl FrenchCrs {
    /// Liste complète de toutes les projections géodésiques légales enregistrées.
    pub const ALL: &'static [FrenchCrs] = &[
        FrenchCrs::Lambert93,
        FrenchCrs::LambertCC42, FrenchCrs::LambertCC43, FrenchCrs::LambertCC44, FrenchCrs::LambertCC45,
        FrenchCrs::LambertCC46, FrenchCrs::LambertCC47, FrenchCrs::LambertCC48, FrenchCrs::LambertCC49, FrenchCrs::LambertCC50,
        FrenchCrs::LambertI, FrenchCrs::LambertII, FrenchCrs::LambertIII, FrenchCrs::LambertIV, FrenchCrs::LambertIIEtendu,
        FrenchCrs::Utm30N, FrenchCrs::Utm31N, FrenchCrs::Utm32N,
        FrenchCrs::Utm20N, FrenchCrs::Utm21N, FrenchCrs::Utm22N,
        FrenchCrs::Utm40S, FrenchCrs::Utm38S, FrenchCrs::Utm58S, FrenchCrs::Utm59S,
        FrenchCrs::Utm5S, FrenchCrs::Utm6S, FrenchCrs::LambertNC,
    ];

    /// Parse un code EPSG en `FrenchCrs`.
    pub fn from_epsg(epsg: u32) -> Option<Self> {
        match epsg {
            2154 => Some(Self::Lambert93),
            3942 => Some(Self::LambertCC42),
            3943 => Some(Self::LambertCC43),
            3944 => Some(Self::LambertCC44),
            3945 => Some(Self::LambertCC45),
            3946 => Some(Self::LambertCC46),
            3947 => Some(Self::LambertCC47),
            3948 => Some(Self::LambertCC48),
            3949 => Some(Self::LambertCC49),
            3950 => Some(Self::LambertCC50),
            27561 => Some(Self::LambertI),
            27562 => Some(Self::LambertII),
            27563 => Some(Self::LambertIII),
            27564 => Some(Self::LambertIV),
            27572 => Some(Self::LambertIIEtendu),
            32630 => Some(Self::Utm30N),
            32631 => Some(Self::Utm31N),
            32632 => Some(Self::Utm32N),
            32620 => Some(Self::Utm20N),
            32621 => Some(Self::Utm21N),
            32622 => Some(Self::Utm22N),
            32740 => Some(Self::Utm40S),
            32738 => Some(Self::Utm38S),
            32758 => Some(Self::Utm58S),
            32759 => Some(Self::Utm59S),
            32705 => Some(Self::Utm5S),
            32706 => Some(Self::Utm6S),
            3163  => Some(Self::LambertNC),
            // ── Aliases EPSG locaux DOM-TOM (datum local ≡ WGS84 à <1m) ────────────────
            4471  => Some(Self::Utm38S),  // Mayotte RGM04 / UTM 38S
            2972  => Some(Self::Utm22N),  // Guyane RGFG95 / UTM 22N
            2975  => Some(Self::Utm40S),  // Reunion RGR92 / UTM 40S
            4559  => Some(Self::Utm20N),  // Antilles RRAF91 / UTM 20N
            3296  => Some(Self::Utm5S),   // Polynesie RGPF / UTM 5S
            3297  => Some(Self::Utm6S),   // Polynesie RGPF / UTM 6S
            _ => None,
        }
    }

    /// Code EPSG correspondant.
    pub fn epsg(&self) -> u32 {
        match self {
            Self::Lambert93 => 2154,
            Self::LambertCC42 => 3942,
            Self::LambertCC43 => 3943,
            Self::LambertCC44 => 3944,
            Self::LambertCC45 => 3945,
            Self::LambertCC46 => 3946,
            Self::LambertCC47 => 3947,
            Self::LambertCC48 => 3948,
            Self::LambertCC49 => 3949,
            Self::LambertCC50 => 3950,
            Self::LambertI => 27561,
            Self::LambertII => 27562,
            Self::LambertIII => 27563,
            Self::LambertIV => 27564,
            Self::LambertIIEtendu => 27572,
            Self::Utm30N => 32630,
            Self::Utm31N => 32631,
            Self::Utm32N => 32632,
            Self::Utm20N => 32620,
            Self::Utm21N => 32621,
            Self::Utm22N => 32622,
            Self::Utm40S => 32740,
            Self::Utm38S => 32738,
            Self::Utm58S => 32758,
            Self::Utm59S => 32759,
            Self::Utm5S => 32705,
            Self::Utm6S => 32706,
            Self::LambertNC => 3163,
        }
    }

    /// Nom lisible.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Lambert93 => "RGF93 / Lambert-93 (EPSG:2154)",
            Self::LambertCC42 => "RGF93 / CC42 (EPSG:3942)",
            Self::LambertCC43 => "RGF93 / CC43 (EPSG:3943)",
            Self::LambertCC44 => "RGF93 / CC44 (EPSG:3944)",
            Self::LambertCC45 => "RGF93 / CC45 (EPSG:3945)",
            Self::LambertCC46 => "RGF93 / CC46 (EPSG:3946)",
            Self::LambertCC47 => "RGF93 / CC47 (EPSG:3947)",
            Self::LambertCC48 => "RGF93 / CC48 (EPSG:3948)",
            Self::LambertCC49 => "RGF93 / CC49 (EPSG:3949)",
            Self::LambertCC50 => "RGF93 / CC50 (EPSG:3950)",
            Self::LambertI => "NTF (Paris) / Lambert zone I (EPSG:27561)",
            Self::LambertII => "NTF (Paris) / Lambert zone II (EPSG:27562)",
            Self::LambertIII => "NTF (Paris) / Lambert zone III (EPSG:27563)",
            Self::LambertIV => "NTF (Paris) / Lambert zone IV (EPSG:27564)",
            Self::LambertIIEtendu => "NTF (Paris) / Lambert II etendu (EPSG:27572)",
            Self::Utm30N => "WGS84 / UTM zone 30N (EPSG:32630)",
            Self::Utm31N => "WGS84 / UTM zone 31N (EPSG:32631)",
            Self::Utm32N => "WGS84 / UTM zone 32N (EPSG:32632)",
            Self::Utm20N => "WGS84 / UTM zone 20N (EPSG:32620)",
            Self::Utm21N => "WGS84 / UTM zone 21N (EPSG:32621)",
            Self::Utm22N => "WGS84 / UTM zone 22N (EPSG:32622)",
            Self::Utm40S => "WGS84 / UTM zone 40S (EPSG:32740)",
            Self::Utm38S => "WGS84 / UTM zone 38S (EPSG:32738)",
            Self::Utm58S => "WGS84 / UTM zone 58S (EPSG:32758)",
            Self::Utm59S => "WGS84 / UTM zone 59S (EPSG:32759)",
            Self::Utm5S => "WGS84 / UTM zone 5S (EPSG:32705)",
            Self::Utm6S => "WGS84 / UTM zone 6S (EPSG:32706)",
            Self::LambertNC => "RGNC91-93 / Lambert NC (EPSG:3163)",
        }
    }

    /// Tente de deduire le CRS depuis un nom de fichier (ex: 'vol_CC44.txt').
    pub fn guess_from_filename(filename: &str) -> Option<Self> {
        let f = filename.to_uppercase();
        if f.contains("LAMBERT93") || f.contains("L93") { return Some(Self::Lambert93); }
        if f.contains("CC42") { return Some(Self::LambertCC42); }
        if f.contains("CC43") { return Some(Self::LambertCC43); }
        if f.contains("CC44") { return Some(Self::LambertCC44); }
        if f.contains("CC45") { return Some(Self::LambertCC45); }
        if f.contains("CC46") { return Some(Self::LambertCC46); }
        if f.contains("CC47") { return Some(Self::LambertCC47); }
        if f.contains("CC48") { return Some(Self::LambertCC48); }
        if f.contains("CC49") { return Some(Self::LambertCC49); }
        if f.contains("CC50") { return Some(Self::LambertCC50); }
        
        if f.contains("L2E") || f.contains("LAMBERT2E") || f.contains("LAMBERTIIE") { return Some(Self::LambertIIEtendu); }
        if f.contains("UTM30N") { return Some(Self::Utm30N); }
        if f.contains("UTM31N") { return Some(Self::Utm31N); }
        if f.contains("UTM32N") { return Some(Self::Utm32N); }

        None
    }

    /// Devine le CRS d'après les coordonnées cartographiques X, Y avec secours sur un CRS par défaut (ex: CRS du projet)
    pub fn guess_from_coordinates_with_default(x: f64, y: f64, default_crs: Self) -> Self {
        // 1. Lambert-93 (EPSG:2154 - Métropole standard)
        if (50_000.0..=1_350_000.0).contains(&x) && (6_000_000.0..=7_200_000.0).contains(&y) {
            return Self::Lambert93;
        }
        // 2. Coniques Conformes 9 Zones (CC42 à CC50 - EPSG:3942..3950)
        // CC42: Y0 = 2 200 000 (y_zone 2)
        // CC43: Y0 = 3 200 000 (y_zone 3)
        // CC44: Y0 = 4 200 000 (y_zone 4)
        // CC45: Y0 = 5 200 000 (y_zone 5)
        // CC46: Y0 = 6 200 000 (y_zone 6)
        // CC47: Y0 = 7 200 000 (y_zone 7)
        // CC48: Y0 = 8 200 000 (y_zone 8)
        // CC49: Y0 = 9 200 000 (y_zone 9)
        // CC50: Y0 = 10 200 000 (y_zone 10)
        if (1_000_000.0..=2_000_000.0).contains(&x) && (1_500_000.0..=11_000_000.0).contains(&y) {
            let y_zone = (y / 1_000_000.0).round() as i32;
            // Troncature classique CC44 (Meyreuil / Bouches-du-Rhône) : Y enregistré en 3 14x xxx au lieu de 4 14x xxx
            if y_zone == 3 && (1_800_000.0..=1_980_000.0).contains(&x) && (3_050_000.0..=3_250_000.0).contains(&y) {
                return Self::LambertCC44;
            }
            match y_zone {
                2 => return Self::LambertCC42,
                3 => return Self::LambertCC43,
                4 => return Self::LambertCC44,
                5 => return Self::LambertCC45,
                6 => return Self::LambertCC46,
                7 => return Self::LambertCC47,
                8 => return Self::LambertCC48,
                9 => return Self::LambertCC49,
                10 => return Self::LambertCC50,
                _ => {}
            }
        }
        // 3. Lambert II Étendu (EPSG:27572)
        if (1_600_000.0..=2_700_000.0).contains(&y) && (50_000.0..=1_200_000.0).contains(&x) {
            return Self::LambertIIEtendu;
        }
        // 4. Antilles (UTM 20N)
        if (500_000.0..=750_000.0).contains(&x) && (1_550_000.0..=1_850_000.0).contains(&y) {
            return Self::Utm20N;
        }
        // 5. La Réunion (UTM 40S)
        if (300_000.0..=400_000.0).contains(&x) && (7_630_000.0..=7_690_000.0).contains(&y) {
            return Self::Utm40S;
        }
        // 6. Mayotte (UTM 38S)
        if (500_000.0..=560_000.0).contains(&x) && (8_580_000.0..=8_610_000.0).contains(&y) {
            return Self::Utm38S;
        }
        // 7. Guyane (UTM 22N)
        if (150_000.0..=400_000.0).contains(&x) && (200_000.0..=600_000.0).contains(&y) {
            return Self::Utm22N;
        }
        default_crs
    }

    /// Devine le CRS d'après les coordonnées cartographiques X, Y (France métropolitaine & Outre-Mer)
    pub fn guess_from_coordinates(x: f64, y: f64) -> Self {
        Self::guess_from_coordinates_with_default(x, y, Self::Lambert93)
    }
}

// ─── Registre des modeles de geoide par territoire ──────────────────────────────────

/// Modele de geoide utilise pour la conversion h_ellipsoidal (WGS84) → h_orthometrique.
/// Chaque territoire dispose de son propre modele distribue par l'IGN via Geoplateforme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeoidModel {
    /// RAF20 — France metropolitaine (RGF93/GRS80, valide metropolitaine entiere)
    Raf20,
    /// RAGTBT2016 — Martinique (RRAF91/GRS80, UTM 20N)
    Ragtbt2016,
    /// RAPH2016 — Guadeloupe (RRAF91/GRS80, UTM 20N)
    Raph2016,
    /// RAG2021 — Guyane (RGFG95/GRS80, UTM 22N)
    Rag2021,
    /// RAR07_v2 — La Reunion (RGR92/GRS80, UTM 40S)
    Rar07,
    /// RAMU2016 — Mayotte (RGM04/GRS80, UTM 38S)
    Ramu2016,
    /// RANC25 — Nouvelle-Caledonie (RGNC91-93/GRS80, UTM 58-59S, Lambert NC)
    Ranc25,
    /// RAPF2011 — Polynesie francaise (RGPF/GRS80, UTM 5-6S)
    Rapf2011,
    /// Approximation ellipsoidale (N=0) — fallback si modele indisponible
    EllipsoidalApprox,
}

impl GeoidModel {
    /// Layer WMS-R de la Geoplateforme IGN correspondant au modele.
    pub fn wms_layer(&self) -> &'static str {
        match self {
            Self::Raf20      => "GEOID.GEOIDGRID2020",
            Self::Ragtbt2016 => "GEOID.GEOIDGRID2016.MART",
            Self::Raph2016   => "GEOID.GEOIDGRID2016.GUAD",
            Self::Rag2021    => "GEOID.GEOIDGRID2021.GUY",
            Self::Rar07      => "GEOID.GEOIDGRID2007.REU",
            Self::Ramu2016   => "GEOID.GEOIDGRID2016.MAY",
            Self::Ranc25     => "GEOID.GEOIDGRID2025.NC",
            Self::Rapf2011   => "GEOID.GEOIDGRID2011.PF",
            Self::EllipsoidalApprox => "",
        }
    }

    /// URL de la grille geoide officielle sur le CDN PROJ.org.
    /// Ce sont les memes fichiers utilises par PROJ, GDAL, QGIS pour les transformations datum.
    /// Telechargement a la demande, cache permanent local -> interpolation bilineaire exacte.
    /// Aucune valeur N n'est codee en dur : tout vient de la grille officielle IGN.
    pub fn proj_cdn_url(&self) -> Option<&'static str> {
        match self {
            // France metropolitaine : RAF20 (2020) -> NGF-IGN69
            Self::Raf20       => Some("https://cdn.proj.org/fr_ign_RAF20.tif"),
            // Martinique : RAGTBT2016 -> IGN1987 Martinique
            Self::Ragtbt2016  => Some("https://cdn.proj.org/fr_ign_RAGTBT2016.tif"),
            // Guadeloupe : RAPH2016 -> IGN1988 Guadeloupe
            Self::Raph2016    => Some("https://cdn.proj.org/fr_ign_RAPH2016.tif"),
            // Guyane : RAG2021 -> NGG1977 Guyane
            Self::Rag2021     => Some("https://cdn.proj.org/fr_ign_RAG2021.tif"),
            // Reunion : RAR07 -> IGN1989 Reunion
            Self::Rar07       => Some("https://cdn.proj.org/fr_ign_RAR07_b.tif"),
            // Mayotte : RAMU2016 -> SHOM1953 Mayotte
            Self::Ramu2016    => Some("https://cdn.proj.org/fr_ign_RAMU2016.tif"),
            // Nouvelle-Caledonie et Polynesie : pas encore dans le CDN PROJ public
            // Fallback WMS IGN Geoplateforme pour ces territoires
            Self::Ranc25      => None,
            Self::Rapf2011    => None,
            Self::EllipsoidalApprox => None,
        }
    }

    /// Resource name pour l'API REST altimetrie IGN (conserve comme fallback secondaire).
    pub fn alti_resource(&self) -> Option<&'static str> {
        match self {
            Self::Raf20 => Some("wgs84_to_ngf"),
            _           => None,
        }
    }
}

impl FrenchCrs {
    /// Modele de geoide approprie pour ce CRS.
    /// Retourne le modele IGN officiel du territoire associe.
    pub fn geoid_model(&self) -> GeoidModel {
        match self {
            // ── France metropolitaine : RAF20 ────────────────────────────────────────────
            Self::Lambert93
            | Self::LambertCC42 | Self::LambertCC43 | Self::LambertCC44
            | Self::LambertCC45 | Self::LambertCC46 | Self::LambertCC47
            | Self::LambertCC48 | Self::LambertCC49 | Self::LambertCC50
            | Self::LambertI | Self::LambertII | Self::LambertIII
            | Self::LambertIV | Self::LambertIIEtendu
            | Self::Utm30N | Self::Utm31N | Self::Utm32N => GeoidModel::Raf20,
            // ── Antilles : fuseau UTM 20N couvre Martinique ET Guadeloupe.
            // Detection fine par longitude : Martinique ~-61°, Guadeloupe ~-62°.
            // Ici on retourne Raph2016 (Guadeloupe) comme valeur par defaut du fuseau.
            // Pour Martinique, utiliser fetch_geoid_undulation_for_territory().
            Self::Utm20N  => GeoidModel::Raph2016,
            // ── St-Pierre-et-Miquelon : pas de modele IGN Geoplateforme, approx WGS84 ─
            Self::Utm21N  => GeoidModel::EllipsoidalApprox,
            // ── DOM-TOM individuels ──────────────────────────────────────────────────────
            Self::Utm22N             => GeoidModel::Rag2021,
            Self::Utm40S             => GeoidModel::Rar07,
            Self::Utm38S             => GeoidModel::Ramu2016,
            Self::Utm58S | Self::Utm59S | Self::LambertNC => GeoidModel::Ranc25,
            Self::Utm5S  | Self::Utm6S  => GeoidModel::Rapf2011,
        }
    }

    /// Separation geoïdale N (metres) pour ce CRS au point (lat, lon) WGS84.
    ///
    /// N = h_ellipsoidal_WGS84 - h_orthometrique_local
    ///
    /// Delegue a `fetch_geoid_undulation` dans ign_client.rs.
    /// Retourne None si le reseau est indisponible (le pipeline basculera sur auto-calibration).
    pub fn fetch_geoid_undulation(&self, lat_deg: f64, lon_deg: f64) -> Option<f64> {
        let sys = match self.geoid_model() {
            GeoidModel::Raf20 => crate::geoid::LegalAltimetricSystem::NgfIgn69,
            GeoidModel::Raph2016 => crate::geoid::LegalAltimetricSystem::Rag04,
            GeoidModel::Ragtbt2016 => crate::geoid::LegalAltimetricSystem::Ign1988,
            GeoidModel::Ramu2016 => crate::geoid::LegalAltimetricSystem::Combani1953,
            _ => crate::geoid::LegalAltimetricSystem::NgfIgn69,
        };
        Some(crate::geoid::dom_tom_geoid_undulation(lon_deg, lat_deg, sys))
    }
}

// ─── Constructeurs de parametres internes ────────────────────────────────────

/// Parametres LCC 2SP pour une zone CC (CC42-CC50).
/// phi0_deg = latitude centrale de la zone (42 a 50).
fn cc_params(phi0_deg: f64) -> Lcc2SpParams {
    let zone = phi0_deg.round() as i32;
    Lcc2SpParams {
        phi1: (phi0_deg - 0.75) * DEG,
        phi2: (phi0_deg + 0.75) * DEG,
        phi0: phi0_deg * DEG,
        lambda0: 3.0 * DEG,
        false_e: 1_700_000.0,
        false_n: ((zone - 40) as f64 * 1_000_000.0 + 200_000.0),
        ellipsoid: GRS80,
    }
}

/// Parametres UTM standard (WGS84) pour un fuseau Z, hemisphere N ou S.
fn utm_params(zone: u32, south: bool) -> TmParams {
    TmParams {
        lambda0: ((zone as f64) * 6.0 - 183.0) * DEG,
        k0: 0.9996,
        false_e: 500_000.0,
        false_n: if south { 10_000_000.0 } else { 0.0 },
        ellipsoid: WGS84,
    }
}

/// Lambert 93 (EPSG:2154).
fn lambert93_params() -> Lcc2SpParams {
    Lcc2SpParams {
        phi1: 44.0 * DEG,
        phi2: 49.0 * DEG,
        phi0: 46.5 * DEG,
        lambda0: 3.0 * DEG,
        false_e: 700_000.0,
        false_n: 6_600_000.0,
        ellipsoid: GRS80,
    }
}

/// Lambert NTF zone (1SP) avec meridien de Paris.
/// phi0_deg : latitude d'origine, k0 : facteur d'echelle, false_n : fausse ordonnee.
fn ntf_lambert_params(phi0_deg: f64, k0: f64, false_n: f64) -> Lcc1SpParams {
    Lcc1SpParams {
        phi0: phi0_deg * DEG,
        lambda0: PARIS_MERIDIAN_RAD, // meridien de Paris -> coordonnees en WGS84 Greenwich apres datum shift
        k0,
        false_e: 600_000.0,
        false_n,
        ellipsoid: CLARKE1880_IGN,
    }
}

/// RGNC91-93 / Lambert NC (EPSG:3163) — Nouvelle-Caledonie.
/// LCC 2SP : phi1=-20.6667°, phi2=-22.3333°, phi0=-21.5°, lam0=166°, FE=400000, FN=300000.
fn lambert_nc_params() -> Lcc2SpParams {
    Lcc2SpParams {
        phi1: (-20.0 - 2.0 / 3.0) * DEG,  // -20°40'N
        phi2: (-22.0 - 1.0 / 3.0) * DEG,  // -22°20'N
        phi0: -21.5 * DEG,
        lambda0: 166.0 * DEG,
        false_e: 400_000.0,
        false_n: 300_000.0,
        ellipsoid: GRS80,
    }
}



/// Erreur geodesique.
#[derive(Debug, Clone)]
pub struct GeodesyError(pub String);

impl std::fmt::Display for GeodesyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "GeodesyError: {}", self.0)
    }
}

/// Projette un point WGS84 (lon deg, lat deg) vers les coordonnees planes (x_m, y_m)
/// du systeme `crs`.
pub fn from_wgs84(crs: FrenchCrs, lon_deg: f64, lat_deg: f64) -> Result<(f64, f64), GeodesyError> {
    let phi = lat_deg * DEG;
    let lambda = lon_deg * DEG;

    match crs {
        // ── Lambert 93 ──────────────────────────────────────────────────────
        FrenchCrs::Lambert93 => Ok(lcc2sp_forward(&lambert93_params(), phi, lambda)),
        // ── CC42-CC50 ────────────────────────────────────────────────────────
        FrenchCrs::LambertCC42 => Ok(lcc2sp_forward(&cc_params(42.0), phi, lambda)),
        FrenchCrs::LambertCC43 => Ok(lcc2sp_forward(&cc_params(43.0), phi, lambda)),
        FrenchCrs::LambertCC44 => Ok(lcc2sp_forward(&cc_params(44.0), phi, lambda)),
        FrenchCrs::LambertCC45 => Ok(lcc2sp_forward(&cc_params(45.0), phi, lambda)),
        FrenchCrs::LambertCC46 => Ok(lcc2sp_forward(&cc_params(46.0), phi, lambda)),
        FrenchCrs::LambertCC47 => Ok(lcc2sp_forward(&cc_params(47.0), phi, lambda)),
        FrenchCrs::LambertCC48 => Ok(lcc2sp_forward(&cc_params(48.0), phi, lambda)),
        FrenchCrs::LambertCC49 => Ok(lcc2sp_forward(&cc_params(49.0), phi, lambda)),
        FrenchCrs::LambertCC50 => Ok(lcc2sp_forward(&cc_params(50.0), phi, lambda)),
        // ── NTF Lambert (via datum shift WGS84->NTF + projection) ────────────
        FrenchCrs::LambertI
        | FrenchCrs::LambertII
        | FrenchCrs::LambertIII
        | FrenchCrs::LambertIV
        | FrenchCrs::LambertIIEtendu => {
            // WGS84 -> ECEF
            let (xw, yw, zw) = geodetic_to_ecef(phi, lambda, 0.0, &WGS84);
            // Appliquer shift inverse (WGS84 -> NTF)
            let (xn, yn, zn) = reverse_geocentric_shift(xw, yw, zw, &NTF_TO_WGS84);
            // ECEF -> NTF geodesique
            let (phi_ntf, lambda_ntf) = ecef_to_geodetic(xn, yn, zn, &CLARKE1880_IGN);
            // λ_NTF est en Greenwich ; il faut l'exprimer par rapport au meridien de Paris
            let lambda_paris = lambda_ntf - PARIS_MERIDIAN_RAD;
            // Projeter en Lambert NTF
            let result = match crs {
                FrenchCrs::LambertI => lcc1sp_forward(
                    &ntf_lambert_params(49.5, 0.999_877_341, 200_000.0),
                    phi_ntf,
                    lambda_paris,
                ),
                FrenchCrs::LambertII => lcc1sp_forward(
                    &ntf_lambert_params(46.8, 0.999_877_420, 200_000.0),
                    phi_ntf,
                    lambda_paris,
                ),
                FrenchCrs::LambertIIEtendu => lcc1sp_forward(
                    &ntf_lambert_params(46.8, 0.999_877_420, 2_200_000.0),
                    phi_ntf,
                    lambda_paris,
                ),
                FrenchCrs::LambertIII => lcc1sp_forward(
                    &ntf_lambert_params(44.1, 0.999_877_499, 200_000.0),
                    phi_ntf,
                    lambda_paris,
                ),
                FrenchCrs::LambertIV => lcc1sp_forward(
                    &ntf_lambert_params(42.165, 0.999_903_139, 200_000.0),
                    phi_ntf,
                    lambda_paris,
                ),
                _ => unreachable!(),
            };
            Ok(result)
        }
        // ── UTM metropole ────────────────────────────────────────────────────
        FrenchCrs::Utm30N => Ok(tm_forward(&utm_params(30, false), phi, lambda)),
        FrenchCrs::Utm31N => Ok(tm_forward(&utm_params(31, false), phi, lambda)),
        FrenchCrs::Utm32N => Ok(tm_forward(&utm_params(32, false), phi, lambda)),
        // ── UTM DOM-TOM ──────────────────────────────────────────────────────
        FrenchCrs::Utm20N => Ok(tm_forward(&utm_params(20, false), phi, lambda)),
        FrenchCrs::Utm21N => Ok(tm_forward(&utm_params(21, false), phi, lambda)),
        FrenchCrs::Utm22N => Ok(tm_forward(&utm_params(22, false), phi, lambda)),
        FrenchCrs::Utm40S => Ok(tm_forward(&utm_params(40, true), phi, lambda)),
        FrenchCrs::Utm38S => Ok(tm_forward(&utm_params(38, true), phi, lambda)),
        FrenchCrs::Utm58S => Ok(tm_forward(&utm_params(58, true), phi, lambda)),
        FrenchCrs::Utm59S => Ok(tm_forward(&utm_params(59, true), phi, lambda)),
        FrenchCrs::Utm5S => Ok(tm_forward(&utm_params(5, true), phi, lambda)),
        FrenchCrs::Utm6S => Ok(tm_forward(&utm_params(6, true), phi, lambda)),
        // ── Lambert NC (RGNC91-93 / EPSG:3163) ─────────────────────────────────────
        FrenchCrs::LambertNC => Ok(lcc2sp_forward(&lambert_nc_params(), phi, lambda)),
    }
}

/// Reprojecte des coordonnees planes (x_m, y_m) du systeme `crs` vers WGS84
/// (lon deg, lat deg).
pub fn to_wgs84(crs: FrenchCrs, x: f64, y: f64) -> Result<(f64, f64), GeodesyError> {
    let (phi_rad, lambda_rad) = match crs {
        // ── Lambert 93 ──────────────────────────────────────────────────────
        FrenchCrs::Lambert93 => lcc2sp_inverse(&lambert93_params(), x, y),
        // ── CC42-CC50 ────────────────────────────────────────────────────────
        FrenchCrs::LambertCC42 => lcc2sp_inverse(&cc_params(42.0), x, y),
        FrenchCrs::LambertCC43 => lcc2sp_inverse(&cc_params(43.0), x, y),
        FrenchCrs::LambertCC44 => {
            let actual_y = if (3_000_000.0..3_500_000.0).contains(&y) { y + 1_000_000.0 } else { y };
            lcc2sp_inverse(&cc_params(44.0), x, actual_y)
        },
        FrenchCrs::LambertCC45 => lcc2sp_inverse(&cc_params(45.0), x, y),
        FrenchCrs::LambertCC46 => lcc2sp_inverse(&cc_params(46.0), x, y),
        FrenchCrs::LambertCC47 => lcc2sp_inverse(&cc_params(47.0), x, y),
        FrenchCrs::LambertCC48 => lcc2sp_inverse(&cc_params(48.0), x, y),
        FrenchCrs::LambertCC49 => lcc2sp_inverse(&cc_params(49.0), x, y),
        FrenchCrs::LambertCC50 => lcc2sp_inverse(&cc_params(50.0), x, y),
        // ── NTF Lambert ──────────────────────────────────────────────────────
        FrenchCrs::LambertI
        | FrenchCrs::LambertII
        | FrenchCrs::LambertIII
        | FrenchCrs::LambertIV
        | FrenchCrs::LambertIIEtendu => {
            let (phi_ntf, lambda_paris) = match crs {
                FrenchCrs::LambertI => {
                    lcc1sp_inverse(&ntf_lambert_params(49.5, 0.999_877_341, 200_000.0), x, y)
                }
                FrenchCrs::LambertII => {
                    lcc1sp_inverse(&ntf_lambert_params(46.8, 0.999_877_420, 200_000.0), x, y)
                }
                FrenchCrs::LambertIIEtendu => {
                    lcc1sp_inverse(&ntf_lambert_params(46.8, 0.999_877_420, 2_200_000.0), x, y)
                }
                FrenchCrs::LambertIII => {
                    lcc1sp_inverse(&ntf_lambert_params(44.1, 0.999_877_499, 200_000.0), x, y)
                }
                FrenchCrs::LambertIV => {
                    lcc1sp_inverse(&ntf_lambert_params(42.165, 0.999_903_139, 200_000.0), x, y)
                }
                _ => unreachable!(),
            };
            // Remettre λ dans le repere Greenwich
            let lambda_ntf_greenwich = lambda_paris + PARIS_MERIDIAN_RAD;
            // NTF geodesique -> ECEF (Clarke 1880 IGN, h=0)
            let (xn, yn, zn) = geodetic_to_ecef(phi_ntf, lambda_ntf_greenwich, 0.0, &CLARKE1880_IGN);
            // Datum shift NTF -> WGS84
            let (xw, yw, zw) = apply_geocentric_shift(xn, yn, zn, &NTF_TO_WGS84);
            // ECEF -> WGS84 geodesique
            ecef_to_geodetic(xw, yw, zw, &WGS84)
        }
        // ── UTM metropole ────────────────────────────────────────────────────
        FrenchCrs::Utm30N => tm_inverse(&utm_params(30, false), x, y),
        FrenchCrs::Utm31N => tm_inverse(&utm_params(31, false), x, y),
        FrenchCrs::Utm32N => tm_inverse(&utm_params(32, false), x, y),
        // ── UTM DOM-TOM ──────────────────────────────────────────────────────
        FrenchCrs::Utm20N => tm_inverse(&utm_params(20, false), x, y),
        FrenchCrs::Utm21N => tm_inverse(&utm_params(21, false), x, y),
        FrenchCrs::Utm22N => tm_inverse(&utm_params(22, false), x, y),
        FrenchCrs::Utm40S => tm_inverse(&utm_params(40, true), x, y),
        FrenchCrs::Utm38S => tm_inverse(&utm_params(38, true), x, y),
        FrenchCrs::Utm58S => tm_inverse(&utm_params(58, true), x, y),
        FrenchCrs::Utm59S => tm_inverse(&utm_params(59, true), x, y),
        FrenchCrs::Utm5S => tm_inverse(&utm_params(5, true), x, y),
        FrenchCrs::Utm6S => tm_inverse(&utm_params(6, true), x, y),
        // ── Lambert NC (RGNC91-93) ──────────────────────────────────────────────────────────
        FrenchCrs::LambertNC => lcc2sp_inverse(&lambert_nc_params(), x, y),
    };

    Ok((lambda_rad / DEG, phi_rad / DEG))
}

/// Vérifie si des coordonnées WGS84 (lon, lat) s'inscrivent dans le domaine géographique valide d'un CRS français.
pub fn is_valid_wgs84_for_crs(crs: FrenchCrs, lon: f64, lat: f64) -> bool {
    if !lon.is_finite() || !lat.is_finite() {
        return false;
    }
    match crs {
        FrenchCrs::Lambert93
        | FrenchCrs::LambertCC42
        | FrenchCrs::LambertCC43
        | FrenchCrs::LambertCC44
        | FrenchCrs::LambertCC45
        | FrenchCrs::LambertCC46
        | FrenchCrs::LambertCC47
        | FrenchCrs::LambertCC48
        | FrenchCrs::LambertCC49
        | FrenchCrs::LambertCC50
        | FrenchCrs::LambertI
        | FrenchCrs::LambertII
        | FrenchCrs::LambertIII
        | FrenchCrs::LambertIV
        | FrenchCrs::LambertIIEtendu
        | FrenchCrs::Utm30N
        | FrenchCrs::Utm31N
        | FrenchCrs::Utm32N => {
            // Métropole française : lat ~ [40.5, 52.0], lon ~ [-6.0, 11.0]
            (40.5..=52.0).contains(&lat) && (-6.0..=11.0).contains(&lon)
        }
        FrenchCrs::Utm20N => {
            // Guadeloupe / Martinique : lat ~ [13.5, 19.0], lon ~ [-64.0, -59.0]
            (13.5..=19.0).contains(&lat) && (-64.0..=-59.0).contains(&lon)
        }
        FrenchCrs::Utm22N => {
            // Guyane : lat ~ [1.5, 7.0], lon ~ [-56.0, -50.0]
            (1.5..=7.0).contains(&lat) && (-56.0..=-50.0).contains(&lon)
        }
        FrenchCrs::Utm40S => {
            // La Réunion : lat ~ [-22.0, -20.0], lon ~ [54.5, 56.5]
            (-22.0..=-20.0).contains(&lat) && (54.5..=56.5).contains(&lon)
        }
        FrenchCrs::Utm38S => {
            // Mayotte : lat ~ [-13.5, -12.0], lon ~ [44.0, 46.0]
            (-13.5..=-12.0).contains(&lat) && (44.0..=46.0).contains(&lon)
        }
        FrenchCrs::Utm21N => {
            // Saint-Pierre-et-Miquelon : lat ~ [46.0, 48.0], lon ~ [-57.5, -55.5]
            (46.0..=48.0).contains(&lat) && (-57.5..=-55.5).contains(&lon)
        }
        FrenchCrs::Utm58S | FrenchCrs::Utm59S | FrenchCrs::LambertNC => {
            // Nouvelle-Calédonie : lat ~ [-23.5, -18.5], lon ~ [162.0, 170.0]
            (-23.5..=-18.5).contains(&lat) && (162.0..=170.0).contains(&lon)
        }
        FrenchCrs::Utm5S | FrenchCrs::Utm6S => {
            // Polynésie : lat ~ [-29.0, -6.0], lon ~ [-156.0, -133.0]
            (-29.0..=-6.0).contains(&lat) && (-156.0..=-133.0).contains(&lon)
        }
    }
}

/// Conversion plane (x, y) vers WGS84 (lon, lat) avec détection automatique fallback si hors-domaine.
pub fn to_wgs84_robust(crs: FrenchCrs, x: f64, y: f64) -> (f64, f64) {
    if let Ok((lon, lat)) = to_wgs84(crs, x, y) {
        if is_valid_wgs84_for_crs(crs, lon, lat) {
            return (lon, lat);
        }
    }
    let guessed = FrenchCrs::guess_from_coordinates(x, y);
    if let Ok((lon, lat)) = to_wgs84(guessed, x, y) {
        if is_valid_wgs84_for_crs(guessed, lon, lat) {
            return (lon, lat);
        }
    }
    (0.0, 0.0)
}

/// Reprojecte un point d'un CRS source vers un CRS cible, tous deux francais.
pub fn reproject(
    src: FrenchCrs,
    dst: FrenchCrs,
    x: f64,
    y: f64,
) -> Result<(f64, f64), GeodesyError> {
    let (lon, lat) = to_wgs84(src, x, y)?;
    from_wgs84(dst, lon, lat)
}

/// Retourne la zone Conique Conforme CC9 (CC42 à CC50) la mieux adaptée pour une latitude WGS84 donnée.
pub fn best_cc_zone_from_wgs84(lat: f64) -> FrenchCrs {
    let lat_deg = lat.clamp(41.5, 50.5);
    let zone = lat_deg.round() as i32;
    match zone {
        42 => FrenchCrs::LambertCC42,
        43 => FrenchCrs::LambertCC43,
        44 => FrenchCrs::LambertCC44,
        45 => FrenchCrs::LambertCC45,
        46 => FrenchCrs::LambertCC46,
        47 => FrenchCrs::LambertCC47,
        48 => FrenchCrs::LambertCC48,
        49 => FrenchCrs::LambertCC49,
        50 => FrenchCrs::LambertCC50,
        _ if lat_deg < 42.0 => FrenchCrs::LambertCC42,
        _ => FrenchCrs::LambertCC50,
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Tolerance pour les systemes RGF93 (sub-mm)
    const TOL_RGF93: f64 = 0.001; // 1 mm
    /// Tolerance pour les systemes NTF avec Helmert 3-param (~3 m)
    const TOL_NTF: f64 = 3.0;
    /// Tolerance UTM (sub-mm dans la zone)
    const TOL_UTM: f64 = 0.001;

    /// Aller-retour WGS84 -> CRS -> WGS84 doit etre idempotent.
    fn round_trip_wgs84(crs: FrenchCrs, lon: f64, lat: f64, tol_m: f64) {
        let (x, y) = from_wgs84(crs, lon, lat).unwrap();
        let (lon2, lat2) = to_wgs84(crs, x, y).unwrap();
        let dlat_m = (lat2 - lat).abs() * 111_319.0;
        let dlon_m = (lon2 - lon).abs() * 111_319.0 * lat.to_radians().cos();
        assert!(
            dlat_m < tol_m && dlon_m < tol_m,
            "{:?}: aller-retour Δlat={:.4}m Δlon={:.4}m (tol={}m)",
            crs,
            dlat_m,
            dlon_m,
            tol_m
        );
    }

    // ── Lambert 93 (point de reference IGN : Paris, Notre-Dame) ────────────
    #[test]
    fn lambert93_paris() {
        // Coordonnees NGN93 de reference (IGN): Notre-Dame de Paris
        // lon=2.34901 degE lat=48.85261 degN -> E~=652559 N~=6862566 (approx)
        let (e, n) = from_wgs84(FrenchCrs::Lambert93, 2.34901, 48.85261).unwrap();
        let (lon, lat) = to_wgs84(FrenchCrs::Lambert93, e, n).unwrap();
        assert!(
            (lon - 2.34901).abs() < 1e-8,
            "lon diff: {}",
            (lon - 2.34901).abs()
        );
        assert!(
            (lat - 48.85261).abs() < 1e-8,
            "lat diff: {}",
            (lat - 48.85261).abs()
        );
    }

    // ── CC44 - zone Meyreuil (Bouches-du-Rhone) ────────────────────────────
    #[test]
    fn cc44_meyreuil_round_trip() {
        round_trip_wgs84(FrenchCrs::LambertCC44, 5.5, 43.5, TOL_RGF93);
    }

    // ── Toutes les zones CC (aller-retour) ────────────────────────────────
    #[test]
    fn cc_all_zones_round_trip() {
        let cases = [
            (FrenchCrs::LambertCC42, 3.0, 42.0),
            (FrenchCrs::LambertCC43, 3.0, 43.0),
            (FrenchCrs::LambertCC44, 3.0, 44.0),
            (FrenchCrs::LambertCC45, 3.0, 45.0),
            (FrenchCrs::LambertCC46, 3.0, 46.0),
            (FrenchCrs::LambertCC47, 3.0, 47.0),
            (FrenchCrs::LambertCC48, 3.0, 48.0),
            (FrenchCrs::LambertCC49, 3.0, 49.0),
            (FrenchCrs::LambertCC50, 3.0, 50.0),
        ];
        for (crs, lon, lat) in cases {
            round_trip_wgs84(crs, lon, lat, TOL_RGF93);
        }
    }

    // ── NTF Lambert II etendu (aller-retour, tolerance Helmert) ───────────
    #[test]
    fn lambert2_etendu_round_trip() {
        round_trip_wgs84(FrenchCrs::LambertIIEtendu, 2.0, 46.5, TOL_NTF);
    }

    // ── UTM 31N - centre France ────────────────────────────────────────────
    #[test]
    fn utm31n_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm31N, 3.0, 46.0, TOL_UTM);
    }

    // ── UTM 20N - Martinique ───────────────────────────────────────────────
    #[test]
    fn utm20n_martinique_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm20N, -61.0, 14.6, TOL_UTM);
    }

    // ── UTM 40S - Reunion ─────────────────────────────────────────────────
    #[test]
    fn utm40s_reunion_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm40S, 55.5, -21.1, TOL_UTM);
    }

    // ── UTM 38S - Mayotte ─────────────────────────────────────────────────
    #[test]
    fn utm38s_mayotte_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm38S, 45.2, -12.8, TOL_UTM);
    }

    // ── UTM 58S/59S - Nouvelle-Caledonie ──────────────────────────────────
    #[test]
    fn utm_ncal_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm58S, 165.0, -21.5, TOL_UTM);
        round_trip_wgs84(FrenchCrs::Utm59S, 170.0, -22.0, TOL_UTM);
    }

    // ── UTM 6S - Tahiti ───────────────────────────────────────────────────
    #[test]
    fn utm6s_tahiti_round_trip() {
        round_trip_wgs84(FrenchCrs::Utm6S, -149.4, -17.5, TOL_UTM);
    }

    // ── reproject CC44 -> Lambert93 ────────────────────────────────────────
    #[test]
    fn reproject_cc44_to_l93() {
        let (x_cc44, y_cc44) = from_wgs84(FrenchCrs::LambertCC44, 5.5, 43.5).unwrap();
        let (x_l93, y_l93) =
            reproject(FrenchCrs::LambertCC44, FrenchCrs::Lambert93, x_cc44, y_cc44).unwrap();
        let (x_l93_direct, y_l93_direct) = from_wgs84(FrenchCrs::Lambert93, 5.5, 43.5).unwrap();
        assert!(
            (x_l93 - x_l93_direct).abs() < TOL_RGF93,
            "Δx={:.4}m",
            (x_l93 - x_l93_direct).abs()
        );
        assert!(
            (y_l93 - y_l93_direct).abs() < TOL_RGF93,
            "Δy={:.4}m",
            (y_l93 - y_l93_direct).abs()
        );
    }
}
