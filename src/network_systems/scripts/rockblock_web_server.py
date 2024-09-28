import http.server
import logging
import random
import signal
import socketserver
import sys
from urllib import parse

# Define port and error code descriptions
PORT = 8100

error_codes = {
    10: "Invalid login credentials",
    11: "No RockBLOCK with this IMEI found on your account",
    12: "RockBLOCK has no line rental",
    13: "Your account has insufficient credit",
    14: "Could not decode hex data",
    15: "Data too long",
    16: "No data",
    99: "System Error",
}

# Set up logging
logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)


# Custom HTTP request handler
class MyHandler(http.server.BaseHTTPRequestHandler):

    def do_POST(self):
        logger.debug("Received POST request.")
        # Extract data parameter from the request
        data = self.get_data()
        logger.debug(f"Extracted data: {data}")

        # Check if data is not empty
        if data:
            try:
                # Decode the hex data
                error_code = int(data, 16)
                logger.debug(f"Decoded error code: {error_code}")
            except ValueError:
                # If decoding fails, return error code 99
                logger.error("Failed to decode data. Using error code 99.")
                error_code = 99
        else:
            # Default error code if data is empty
            logger.warning("No data found. Using error code 99.")
            error_code = 99

        # Handle error codes and special case for error_code 0
        if error_code == 0:
            random_number = random.randint(1, 10000)  # Generate a random positive integer
            response = f'OK,"{random_number}"'
            logger.info(f"Generated response: {response}")
        elif error_code in error_codes:
            response = f"FAILED,{error_code},{error_codes[error_code]}"
            logger.info(f"Generated response: {response}")
        else:
            response = f"FAILED,99,{error_codes[99]}"
            logger.info(f"Generated response: {response}")

        # Send response status code
        self.send_response(200)

        # Send headers
        self.send_header("Content-type", "text/plain")
        self.end_headers()

        # Write the response content
        self.wfile.write(response.encode("utf-8"))

    def get_data(self):
        """Extracts the data parameter from the URL query string (e.g., ?data=48656C6C6F)"""
        query = parse.urlsplit(self.path).query
        params = dict(parse.parse_qsl(query))
        return params.get("data", "")


# Set up the server
def run_server():
    with socketserver.TCPServer(("", PORT), MyHandler) as httpd:
        logger.info(f"Serving on port {PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            logger.info("Server is shutting down...")


# Handle graceful shutdown
def signal_handler(signal, frame):
    logger.info("Received exit signal, shutting down...")
    sys.exit(0)


# Set up signal handling
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
    run_server()
