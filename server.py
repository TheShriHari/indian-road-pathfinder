#!/usr/bin/env python3
import http.server
import socketserver
import os
import sys
import webbrowser

PORT = 8000
DIRECTORY = os.path.join(os.path.dirname(__file__), "web_simulation")

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

def run_server():
    os.chdir(DIRECTORY)
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print(f"========================================================")
        print(f" PATHFINDER STUDIO LOCAL SERVER RUNNING                ")
        print(f" URL: http://localhost:{PORT}                          ")
        print(f" Press Ctrl+C to stop server                           ")
        print(f"========================================================")
        webbrowser.open(f"http://localhost:{PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")
            sys.exit(0)

if __name__ == "__main__":
    run_server()
