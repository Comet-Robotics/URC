#![warn(rust_2018_idioms)]
#![allow(dead_code)]

use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::Arc;

use anyhow::Result;
use base64::prelude::BASE64_STANDARD;
use base64::Engine;

use tokio::sync::{mpsc, Mutex};

/// must_read_stdin blocks until input is received from stdin
#[allow(clippy::assigning_clones)]
pub fn must_read_stdin() -> Result<String> {
    let mut line = String::new();

    std::io::stdin().read_line(&mut line)?;
    line = line.trim().to_owned();
    println!();

    Ok(line)
}

// Allows compressing offer/answer to bypass terminal input limits.
// const COMPRESS: bool = false;

/// encode encodes the input in base64
/// It can optionally zip the input before encoding
pub fn encode(b: &str) -> String {
    //if COMPRESS {
    //    b = zip(b)
    //}

    BASE64_STANDARD.encode(b)
}

/// decode decodes the input from base64
/// It can optionally unzip the input after decoding
pub fn decode(s: &str) -> Result<String> {
    let b = BASE64_STANDARD.decode(s)?;

    //if COMPRESS {
    //    b = unzip(b)
    //}

    let s = String::from_utf8(b)?;
    Ok(s)
}
/*
func zip(in []byte) []byte {
    var b bytes.Buffer
    gz := gzip.NewWriter(&b)
    _, err := gz.Write(in)
    if err != nil {
        panic(err)
    }
    err = gz.Flush()
    if err != nil {
        panic(err)
    }
    err = gz.Close()
    if err != nil {
        panic(err)
    }
    return b.Bytes()
}

func unzip(in []byte) []byte {
    var b bytes.Buffer
    _, err := b.Write(in)
    if err != nil {
        panic(err)
    }
    r, err := gzip.NewReader(&b)
    if err != nil {
        panic(err)
    }
    res, err := ioutil.ReadAll(r)
    if err != nil {
        panic(err)
    }
    return res
}
*/