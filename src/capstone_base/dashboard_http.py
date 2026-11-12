"""Common HTTP response behavior for standalone and combined dashboards."""

import json
from http.server import BaseHTTPRequestHandler


class DashboardHandler(BaseHTTPRequestHandler):
    frame_ancestors = "'none'"

    def log_message(self, *args):
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(2)

    def reply(self, status, data, content_type="application/json"):
        if content_type == "application/json":
            data = json.dumps(data, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; style-src 'self'; script-src 'self'; "
            f"connect-src 'self'; frame-src 'self'; frame-ancestors {self.frame_ancestors}; "
            "base-uri 'none'",
        )
        self.end_headers()
        self.wfile.write(data)
