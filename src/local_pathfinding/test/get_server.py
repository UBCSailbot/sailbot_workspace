"""
This is a basic http server to handle GET requests from the global path module.

It returns a GPS position.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

TEST_PORT = 3005
GET_TEST_URL = "http://localhost:" + str(TEST_PORT) + "/api/gps"


class CustomRequestHandler(BaseHTTPRequestHandler):
    def _set_response(self, status_code=200, content_type="application/json"):
        self.send_response(status_code)
        self.send_header("Content-type", content_type)
        self.end_headers()

    def do_GET(self):
        data = {"latitude":49.34175775635472, "longitude":-123.35453636335373}
        json_data = json.dumps(data).encode("utf-8")
        self._set_response(200)
        self.wfile.write(json_data)


def run_server(port=TEST_PORT) -> HTTPServer:
    server_address = ("localhost", port)
    httpd = HTTPServer(server_address, CustomRequestHandler)

    def run():
        print(f"Server running on http://localhost:{port}")
        httpd.serve_forever()

    # Start the server in a separate thread
    server_thread = threading.Thread(target=run)
    server_thread.start()

    return httpd


def shutdown_server(httpd: HTTPServer):
    httpd.shutdown()


if __name__ == "__main__":
    run_server()
