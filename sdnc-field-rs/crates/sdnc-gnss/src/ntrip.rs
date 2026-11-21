//! Client NTRIP v1/v2 TCP en Rust pur (sourcetable + authentification Basic + flux RTCM3).

use crate::types::NtripConfig;
use anyhow::{anyhow, Context, Result};
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct NtripMountpoint {
    pub name: String,
    pub identifier: String,
    pub format: String,
    pub nav_system: String,
    pub country: String,
    pub lat: f64,
    pub lon: f64,
}

fn base64_encode(input: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(input.len().div_ceil(3) * 4);
    for chunk in input.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = if chunk.len() > 1 { chunk[1] as u32 } else { 0 };
        let b2 = if chunk.len() > 2 { chunk[2] as u32 } else { 0 };
        let n = (b0 << 16) | (b1 << 8) | b2;
        out.push(TABLE[((n >> 18) & 63) as usize] as char);
        out.push(TABLE[((n >> 12) & 63) as usize] as char);
        out.push(if chunk.len() > 1 {
            TABLE[((n >> 6) & 63) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            TABLE[(n & 63) as usize] as char
        } else {
            '='
        });
    }
    out
}

/// Récupère la liste des mountpoints disponibles (Sourcetable) sur un caster NTRIP.
pub fn fetch_ntrip_sourcetable(host: &str, port: u16) -> Result<Vec<NtripMountpoint>> {
    let addr = format!("{}:{}", host, port);
    let mut stream = TcpStream::connect(&addr)
        .with_context(|| format!("Connexion NTRIP impossible vers {}", addr))?;
    stream.set_read_timeout(Some(Duration::from_secs(5)))?;
    stream.set_write_timeout(Some(Duration::from_secs(5)))?;

    let req = format!(
        "GET / HTTP/1.0\r\nHost: {}\r\nUser-Agent: NTRIP SDNCQField-RS/0.1\r\nAccept: */*\r\nConnection: close\r\n\r\n",
        host
    );
    stream.write_all(req.as_bytes())?;

    let mut response = String::new();
    let _ = stream.read_to_string(&mut response);

    let mut mounts = Vec::new();
    for line in response.lines() {
        if line.starts_with("STR;") {
            let cols: Vec<&str> = line.split(';').collect();
            if cols.len() >= 11 {
                mounts.push(NtripMountpoint {
                    name: cols[1].to_string(),
                    identifier: cols[2].to_string(),
                    format: cols[3].to_string(),
                    nav_system: cols[6].to_string(),
                    country: cols[8].to_string(),
                    lat: cols[9].parse().unwrap_or(0.0),
                    lon: cols[10].parse().unwrap_or(0.0),
                });
            }
        }
    }
    Ok(mounts)
}

/// Ouvre une connexion NTRIP active sur le mountpoint configuré, prête à lire les trames RTCM3.
pub fn connect_ntrip_stream(cfg: &NtripConfig) -> Result<TcpStream> {
    let addr = format!("{}:{}", cfg.host, cfg.port);
    let mut stream = TcpStream::connect(&addr)
        .with_context(|| format!("Connexion NTRIP impossible vers {}", addr))?;
    stream.set_read_timeout(Some(Duration::from_secs(4)))?;
    stream.set_write_timeout(Some(Duration::from_secs(4)))?;

    let auth = base64_encode(format!("{}:{}", cfg.username, cfg.password).as_bytes());
    let req = format!(
        "GET /{} HTTP/1.0\r\nHost: {}\r\nUser-Agent: NTRIP SDNCQField-RS/0.1\r\nAuthorization: Basic {}\r\n\r\n",
        cfg.mountpoint.trim_start_matches('/'),
        cfg.host,
        auth
    );
    stream.write_all(req.as_bytes())?;

    let mut header_buf = [0u8; 256];
    let n = stream.read(&mut header_buf)?;
    let header_str = String::from_utf8_lossy(&header_buf[..n]);
    if !header_str.contains("200") {
        return Err(anyhow!(
            "Refus du caster NTRIP ({}) : {}",
            cfg.mountpoint,
            header_str.lines().next().unwrap_or("réponse vide")
        ));
    }

    stream.set_read_timeout(Some(Duration::from_millis(250)))?;
    Ok(stream)
}
