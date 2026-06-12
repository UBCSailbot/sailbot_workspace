import argparse
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


# Store the last POST body so tests can query it via GET /last
_last_body_lock = threading.Lock()
_last_body: bytes = b""


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Serve the last POSTed body at /last for test introspection
        if self.path == "/last":
            with _last_body_lock:
                body = _last_body
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # Default empty OK for any other GET
        self.send_response(200)
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_POST(self):
        self.send_response(200)
        self.end_headers()
        content_len = int(self.headers["Content-Length"])
        body = self.rfile.read(content_len)
        with _last_body_lock:
            global _last_body
            _last_body = body
        print(body)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="127.0.0.1")
    parser.add_argument("-p", "--port", type=int, default=8081)
    args = parser.parse_args()

    print(f"Running HTTP Echo Server at http://{args.host}:{args.port}")

    with HTTPServer((args.host, args.port), HTTPRequestHandler) as server:
        server.serve_forever()


if __name__ == "__main__":
    main()
