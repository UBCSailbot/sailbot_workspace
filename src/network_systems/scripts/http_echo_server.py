import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer


class HTTPRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.end_headers()

    def do_POST(self):
        self.send_response(200)
        self.end_headers()
        content_len = int(self.headers["Content-Length"])
        body = self.rfile.read(content_len)
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
