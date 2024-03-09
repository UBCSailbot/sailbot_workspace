"""
This is a basic http server to handle POST requests from the global path module until the NET
endpoint is implemented.

It receives a JSON payload with a list of waypoints and prints them to the console.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer


class CustomRequestHandler(BaseHTTPRequestHandler):
    def _set_response(self, status_code=200, content_type="application/json"):
        self.send_response(status_code)
        self.send_header("Content-type", content_type)
        self.end_headers()

    def do_POST(self):
        content_length = int(self.headers["Content-Length"])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode("utf-8"))

        # Process the data as needed
        waypoints = data.get("waypoints", [])

        # For now, just print the waypoints
        print("Received waypoints:", waypoints)

        self._set_response(200)
        self.wfile.write(
            json.dumps({"message": "Global path received successfully"}).encode("utf-8")
        )


def run_server(port=8081) -> HTTPServer:
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
