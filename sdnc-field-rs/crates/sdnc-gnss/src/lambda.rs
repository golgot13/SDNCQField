//! Algorithme LAMBDA / MLAMBDA de résolution d'ambiguïtés entières de phase porteuse.
//!
//! Portage direct de `lambda.c` de RTKLIB (Teunissen 1995 / Chang et al. 2005) :
//! 1. Factorisation $Q = L^T D L$
//! 2. Décorrélation entière unimodulaire $Z$ (réduction de Gauss + permutations)
//! 3. Recherche séquentielle branch-and-bound des 2 meilleurs candidats entiers $(z_1, z_2)$
//! 4. Transformation inverse $a = Z^{-T} z$ et Ratio Test $s_2 / s_1$

use anyhow::{anyhow, Result};
use nalgebra::{DMatrix, DVector};

#[derive(Debug, Clone)]
pub struct LambdaSolution {
    /// Meilleur vecteur d'ambiguïtés entières résolu
    pub best_int: DVector<f64>,
    /// Second meilleur vecteur entier
    pub second_int: DVector<f64>,
    /// Norme quadratique du meilleur candidat `s[0]`
    pub best_dist: f64,
    /// Norme quadratique du second candidat `s[1]`
    pub second_dist: f64,
    /// Ratio Test `s[1] / s[0]` (Fix valide généralement si >= 3.0)
    pub ratio: f64,
}

/// Résout le problème des moindres carrés en nombres entiers (ILS) :
/// $\min_{a \in \mathbb{Z}^n} (a - \hat{a})^T Q^{-1} (a - \hat{a})$
pub fn lambda_resolve(a_float: &DVector<f64>, q_mat: &DMatrix<f64>) -> Result<LambdaSolution> {
    let n = a_float.len();
    if n == 0 || q_mat.nrows() != n || q_mat.ncols() != n {
        return Err(anyhow!("Dimensions invalides pour LAMBDA (n={})", n));
    }

    // Symétrisation numérique de Q
    let q_sym = (q_mat + q_mat.transpose()) * 0.5;
    let (mut l, mut d) = ld_factor(&q_sym)?;
    let mut z = DMatrix::<f64>::identity(n, n);

    // Réduction de Gauss + permutations (décorrélation)
    reduction(n, &mut l, &mut d, &mut z);

    // Transformation z_float = Z^T * a_float
    let z_float = z.transpose() * a_float;

    // Recherche séquentielle des 2 meilleurs candidats entiers
    let (zn, s) = search_mlambda(n, &l, &d, &z_float)?;

    // Transformation inverse : a = (Z^T)^-1 * zn
    let zt = z.transpose();
    let zt_inv = zt
        .try_inverse()
        .ok_or_else(|| anyhow!("Matrice unimodulaire Z non inversible"))?;

    let mut best = &zt_inv * &zn[0];
    let mut second = &zt_inv * &zn[1];
    for i in 0..n {
        best[i] = best[i].round();
        second[i] = second[i].round();
    }

    let ratio = if s[0] > 1e-12 {
        (s[1] / s[0]).min(999.9)
    } else {
        999.9
    };

    Ok(LambdaSolution {
        best_int: best,
        second_int: second,
        best_dist: s[0],
        second_dist: s[1],
        ratio,
    })
}

/// Factorisation $Q = L^T D L$ (L triangulaire inférieure unitaire, D diagonale positive).
fn ld_factor(q: &DMatrix<f64>) -> Result<(DMatrix<f64>, Vec<f64>)> {
    let n = q.nrows();
    let mut a = q.clone();
    let mut l = DMatrix::<f64>::zeros(n, n);
    let mut d = vec![0.0f64; n];

    for i in (0..n).rev() {
        let diag = a[(i, i)];
        if diag <= 1e-15 {
            return Err(anyhow!("Matrice Q non définie positive (pivot {} = {})", i, diag));
        }
        d[i] = diag;
        l[(i, i)] = 1.0;
        for j in 0..i {
            l[(i, j)] = a[(i, j)] / diag;
        }
        for j in 0..i {
            for k in 0..=j {
                a[(j, k)] -= a[(i, j)] * l[(i, k)];
            }
        }
    }
    Ok((l, d))
}

fn gauss_reduce(n: usize, l: &mut DMatrix<f64>, z: &mut DMatrix<f64>, i: usize, j: usize) {
    let mu = l[(i, j)].round();
    if mu != 0.0 {
        for k in i..n {
            l[(k, j)] -= mu * l[(k, i)];
        }
        for k in 0..n {
            z[(k, j)] -= mu * z[(k, i)];
        }
    }
}

fn permute_ldz(
    n: usize,
    l: &mut DMatrix<f64>,
    d: &mut [f64],
    k: usize,
    del: f64,
    z: &mut DMatrix<f64>,
) {
    let eta = d[k] / del;
    let lam = d[k + 1] * l[(k + 1, k)] / del;

    d[k] = eta * d[k + 1];
    d[k + 1] = del;

    for j in 0..k {
        let a0 = l[(k, j)];
        let a1 = l[(k + 1, j)];
        l[(k, j)] = -l[(k + 1, k)] * a0 + a1;
        l[(k + 1, j)] = eta * a0 + lam * a1;
    }
    l[(k + 1, k)] = lam;

    for j in (k + 2)..n {
        let tmp = l[(j, k)];
        l[(j, k)] = l[(j, k + 1)];
        l[(j, k + 1)] = tmp;
    }
    for j in 0..n {
        let tmp = z[(j, k)];
        z[(j, k)] = z[(j, k + 1)];
        z[(j, k + 1)] = tmp;
    }
}

fn reduction(n: usize, l: &mut DMatrix<f64>, d: &mut [f64], z: &mut DMatrix<f64>) {
    if n < 2 {
        return;
    }
    let mut k = n - 2;
    let mut j = n - 2;
    loop {
        if k <= j {
            for i in (k + 1)..n {
                gauss_reduce(n, l, z, i, k);
            }
        }
        let del = d[k] + l[(k + 1, k)] * l[(k + 1, k)] * d[k + 1];
        if del + 1e-6 < d[k + 1] {
            permute_ldz(n, l, d, k, del, z);
            j = k;
            k = n - 2;
        } else {
            if k == 0 {
                break;
            }
            k -= 1;
        }
    }
}

/// Recherche branch-and-bound MLAMBDA des 2 meilleurs vecteurs entiers.
fn search_mlambda(
    n: usize,
    l: &DMatrix<f64>,
    d: &[f64],
    zs: &DVector<f64>,
) -> Result<([DVector<f64>; 2], [f64; 2])> {
    const MAX_LOOP: usize = 10_000;
    let mut s_mat = vec![vec![0.0f64; n]; n];
    let mut dist = vec![0.0f64; n];
    let mut zb = vec![0.0f64; n];
    let mut z = vec![0.0f64; n];
    let mut step = vec![0.0f64; n];

    let mut k = n - 1;
    zb[k] = zs[k];
    z[k] = zb[k].round();
    let mut y = zb[k] - z[k];
    step[k] = if y <= 0.0 { -1.0 } else { 1.0 };

    let mut maxdist = 1e99_f64;
    let mut nn = 0usize;
    let mut zn = [DVector::<f64>::zeros(n), DVector::<f64>::zeros(n)];
    let mut s_best = [f64::MAX, f64::MAX];
    let mut imax = 0usize;

    for _ in 0..MAX_LOOP {
        let newdist = dist[k] + y * y / d[k];
        if newdist < maxdist {
            if k != 0 {
                k -= 1;
                dist[k] = newdist;
                for i in 0..=k {
                    s_mat[k][i] = s_mat[k + 1][i] + (z[k + 1] - zb[k + 1]) * l[(k + 1, i)];
                }
                zb[k] = zs[k] + s_mat[k][k];
                z[k] = zb[k].round();
                y = zb[k] - z[k];
                step[k] = if y <= 0.0 { -1.0 } else { 1.0 };
            } else {
                if nn < 2 {
                    if nn == 0 || newdist > s_best[imax] {
                        imax = nn;
                    }
                    for i in 0..n {
                        zn[nn][i] = z[i];
                    }
                    s_best[nn] = newdist;
                    nn += 1;
                } else {
                    if newdist < s_best[imax] {
                        for i in 0..n {
                            zn[imax][i] = z[i];
                        }
                        s_best[imax] = newdist;
                        imax = if s_best[0] > s_best[1] { 0 } else { 1 };
                    }
                    maxdist = s_best[imax];
                }
                let z0 = z[0];
                z[0] = z0 + step[0];
                y = zb[0] - z[0];
                step[0] = -step[0] - if step[0] <= 0.0 { -1.0 } else { 1.0 };
            }
        } else if k == n - 1 {
            break;
        } else {
            k += 1;
            z[k] += step[k];
            y = zb[k] - z[k];
            step[k] = -step[k] - if step[k] <= 0.0 { -1.0 } else { 1.0 };
        }
    }

    if nn < 2 {
        return Err(anyhow!("LAMBDA n'a pas trouvé 2 candidats entiers"));
    }
    if s_best[0] > s_best[1] {
        s_best.swap(0, 1);
        zn.swap(0, 1);
    }
    Ok((zn, s_best))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lambda_resolves_correlated_integers() {
        // Exemple classique de Teunissen : matrice fortement corrélée
        let a_float = DVector::from_vec(vec![5.45, 3.10, 2.97]);
        let q = DMatrix::from_row_slice(
            3,
            3,
            &[
                6.290, 5.978, 0.544, //
                5.978, 6.292, 2.340, //
                0.544, 2.340, 6.288, //
            ],
        );
        let q_inv = q.clone().try_inverse().unwrap();

        // Vérification par force brute du vrai minimum global et 2e meilleur entier
        let mut candidates: Vec<(f64, [i32; 3])> = Vec::new();
        for x in 1..=9 {
            for y in 0..=8 {
                for z in 0..=8 {
                    let da = DVector::from_vec(vec![
                        x as f64 - a_float[0],
                        y as f64 - a_float[1],
                        z as f64 - a_float[2],
                    ]);
                    let d2 = (da.transpose() * &q_inv * &da)[(0, 0)];
                    candidates.push((d2, [x, y, z]));
                }
            }
        }
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let res = lambda_resolve(&a_float, &q).unwrap();
        let expected_best = DVector::from_vec(vec![
            candidates[0].1[0] as f64,
            candidates[0].1[1] as f64,
            candidates[0].1[2] as f64,
        ]);
        assert_eq!(res.best_int, expected_best);
        assert!((res.best_dist - candidates[0].0).abs() < 1e-6);
        assert!((res.second_dist - candidates[1].0).abs() < 1e-6);
        assert!(res.ratio > 1.0);
    }
}
