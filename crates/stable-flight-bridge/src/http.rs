//! A minimal static-file HTTP server for the metadata QGC downloads: see [`serve`].

use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use log::{debug, warn};

#[cfg(test)]
mod tests;

/// How long a client may take to send its request or read the response.
const CLIENT_TIMEOUT: Duration = Duration::from_secs(5);
/// Longest request head (request line and headers) read; a longer one is cut off.
const MAX_REQUEST_HEAD: u64 = 8 * 1024;

/// The files a server answers with: URL path → body, all JSON.
pub(crate) type Files = Vec<(String, Vec<u8>)>;

/// Serves `files` on `listener` from a thread of its own, one connection at a time, until the
/// process exits. `GET` of a listed path is `200` with the body; any other path is `404`, and
/// any other method `405`. Every response closes its connection. QGC fetches a file once per
/// CRC, so one connection at a time is plenty.
pub(crate) fn serve(listener: TcpListener, files: Files) -> io::Result<JoinHandle<()>> {
    let files = Arc::new(files);
    thread::Builder::new()
        .name("metadata-http".into())
        .spawn(move || {
            for stream in listener.incoming() {
                match stream {
                    Ok(stream) => {
                        if let Err(e) = respond(stream, &files) {
                            debug!("metadata request failed: {e}");
                        }
                    }
                    Err(e) => warn!("metadata server accept failed: {e}"),
                }
            }
        })
}

/// Reads one request from `stream` and answers it from `files`.
fn respond(stream: TcpStream, files: &[(String, Vec<u8>)]) -> io::Result<()> {
    stream.set_read_timeout(Some(CLIENT_TIMEOUT))?;
    stream.set_write_timeout(Some(CLIENT_TIMEOUT))?;
    let mut reader = BufReader::new(stream.take(MAX_REQUEST_HEAD));
    let mut request_line = String::new();
    reader.read_line(&mut request_line)?;
    // Drain the headers, up to the blank line, so the client sees its request consumed.
    let mut header = String::new();
    while reader.read_line(&mut header)? > 0 && header.trim_end() != "" {
        header.clear();
    }

    let mut parts = request_line.split_whitespace();
    let (method, path) = (parts.next().unwrap_or(""), parts.next().unwrap_or(""));
    let (status, body): (&str, &[u8]) = if method != "GET" {
        ("405 Method Not Allowed", b"")
    } else if let Some((_, body)) = files.iter().find(|(p, _)| p == path) {
        ("200 OK", body)
    } else {
        ("404 Not Found", b"")
    };
    debug!("metadata {method} {path}: {status}");

    let mut stream = reader.into_inner().into_inner();
    write!(
        stream,
        "HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
        body.len()
    )?;
    stream.write_all(body)?;
    stream.flush()
}
