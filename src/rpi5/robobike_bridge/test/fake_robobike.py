"""In-process fake of the ROBOBIKE HTTP API used by the bridge tests."""

import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit


class FakeRobobike:
    def __init__(self):
        self.rows = []
        self.paths = []
        self.commands = []
        self.connections = 0
        self.sockets = []
        self.fail_next = None
        self.mot_spd = 60
        self.sv_drv = 0.0
        self.auto_rows = False
        self.time_ms = 1000
        self.ignore_commands = False
        self.lock = threading.Lock()
        robot = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def setup(self):
                super().setup()
                with robot.lock:
                    robot.connections += 1
                    robot.sockets.append(self.connection)

            def log_message(self, *args):
                pass

            def do_GET(self):
                url = urlsplit(self.path)
                with robot.lock:
                    robot.paths.append(url.path)
                    fail, robot.fail_next = robot.fail_next, None
                    if fail:
                        body, status = "error", fail
                    elif url.path == "/get_acc":
                        if robot.auto_rows:
                            robot.time_ms += 4
                            robot.rows.append(f"a,{robot.time_ms},{robot.sv_drv:.3f},0,0,0,0,0")
                        body, status = "".join(row + "\n" for row in robot.rows), 200
                        robot.rows = []
                    elif url.path == "/clear_buffer":
                        robot.rows = []
                        body, status = "", 200
                    elif url.path == "/command":
                        query = parse_qs(url.query)
                        button = int(query["button"][0])
                        robot.commands.append((button, query.get("value", [None])[0]))
                        if not robot.ignore_commands:
                            robot.apply(button)
                        body = f"b,1042,7,0,0,{robot.mot_spd},0.1,30,0.01,20,40,0.02,0,0,30"
                        status = 200
                    else:
                        body, status = "not found", 404
                data = body.encode()
                self.send_response(status)
                self.send_header("Content-Type", "text/csv")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.server.handle_error = lambda request, address: None  # resets are expected when tests drop links
        self.base_url = f"http://127.0.0.1:{self.server.server_address[1]}"
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True)
        self.thread.start()
        self.closed = False

    def apply(self, button):
        if button == 3 and self.sv_drv == 0:
            self.sv_drv = float(self.mot_spd)
        elif button in (1, 4):
            self.sv_drv = 0.0
        elif button == 8:
            self.sv_drv = -20.0 if self.sv_drv == 0 else 0.0
        elif button == 9:
            self.mot_spd = min(60, self.mot_spd + 1)
        elif button == 10:
            self.mot_spd = max(0, self.mot_spd - 1)

    def queue_rows(self, *rows):
        with self.lock:
            self.rows.extend(rows)

    def close(self):
        if not self.closed:
            self.closed = True
            self.server.shutdown()
            self.server.server_close()
            for sock in self.sockets:  # drop keep-alive connections like a robot going away
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
