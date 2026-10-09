use assert2::check;

use super::*;

/// A server on an ephemeral localhost port with one file at `/1/a.json`; returns its address.
fn server() -> String {
    let listener = TcpListener::bind("127.0.0.1:0").expect("binds");
    let addr = listener.local_addr().expect("bound").to_string();
    serve(listener, vec![("/1/a.json".into(), br#"{"a":1}"#.to_vec())]).expect("spawns");
    addr
}

/// Sends `request` raw and returns the whole response.
fn exchange(addr: &str, request: &str) -> String {
    let mut stream = TcpStream::connect(addr).expect("connects");
    stream.write_all(request.as_bytes()).expect("sends");
    let mut response = String::new();
    stream
        .read_to_string(&mut response)
        .expect("reads to close");
    response
}

#[test]
fn serves_listed_paths_and_refuses_everything_else() {
    let addr = server();
    // (case, request, expected status line, expected body)
    let cases = [
        (
            "listed file",
            "GET /1/a.json HTTP/1.1\r\nHost: x\r\nAccept: */*\r\n\r\n",
            "HTTP/1.1 200 OK",
            r#"{"a":1}"#,
        ),
        (
            "another CRC's path",
            "GET /2/a.json HTTP/1.1\r\n\r\n",
            "HTTP/1.1 404 Not Found",
            "",
        ),
        (
            "not GET",
            "POST /1/a.json HTTP/1.1\r\n\r\n",
            "HTTP/1.1 405 Method Not Allowed",
            "",
        ),
        ("garbage", "\r\n\r\n", "HTTP/1.1 405 Method Not Allowed", ""),
    ];
    // One server for every row: it keeps answering after each response closes.
    for (case, request, status, body) in cases {
        let response = exchange(&addr, request);
        let (head, got_body) = response.split_once("\r\n\r\n").expect("a head");
        check!(head.lines().next() == Some(status), "{case}");
        check!(
            head.contains(&format!("Content-Length: {}", body.len())),
            "{case}"
        );
        check!(head.contains("Connection: close"), "{case}");
        check!(got_body == body, "{case}");
    }
}
