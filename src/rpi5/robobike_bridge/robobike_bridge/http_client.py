"""Keep-alive HTTP client for the ROBOBIKE web API (standard library only)."""

import http.client
from urllib.parse import urlencode, urlsplit

from .drive import ALLOWED_BUTTONS, BT_STR_S

SETTINGS_FIELDS = (
    "PROG_VER", "DATA_VER", "STATUS", "STR0", "MOT_SPD", "GAIN_STR", "GAIN_W_ROLL", "GAIN_DIFF",
    "ANG_STD_NUT", "STR_TURN", "YAW_COEFF", "AUTO_CIRCLING", "STR_CMD_RATE", "STR_CMD_SPD",
)


class RobobikeHttpError(Exception):
    pass


def parse_settings(text):
    """Parse the 'b,...' settings row returned by /command; unknown rows give an empty dict."""
    fields = [field.strip() for field in text.strip().split("\n")[0].split(",")]
    if not fields or fields[0] != "b":
        return {}
    settings = {}
    for name, value in zip(SETTINGS_FIELDS, fields[1:]):
        try:
            settings[name] = int(value)
        except ValueError:
            try:
                settings[name] = float(value)
            except ValueError:
                settings[name] = value
    return settings


class RobobikeHttpClient:
    """One persistent connection; never requests "/" (that would register this host as MASTER)."""

    def __init__(self, base_url, timeout):
        url = urlsplit(base_url)
        if url.scheme != "http" or not url.hostname:
            raise ValueError(f"base_url must be http://host[:port], got {base_url!r}")
        self.host, self.port, self.timeout = url.hostname, url.port or 80, timeout
        self.connection = None

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def get(self, path, query=None):
        if path == "/" or not path.startswith("/"):
            raise ValueError(f"refusing to request {path!r}")
        target = path + ("?" + urlencode(query) if query else "")
        try:
            if self.connection is None:
                self.connection = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout)
            self.connection.request("GET", target)
            response = self.connection.getresponse()
            body = response.read().decode("utf-8", errors="replace")
        except (OSError, http.client.HTTPException) as error:
            self.close()
            raise RobobikeHttpError(f"GET {path}: {error}") from error
        if response.status != 200:
            self.close()
            raise RobobikeHttpError(f"GET {path}: HTTP {response.status}")
        return body

    def get_acc(self):
        return self.get("/get_acc")

    def clear_buffer(self):
        self.get("/clear_buffer")

    def command(self, button, value=None):
        if button not in ALLOWED_BUTTONS:
            raise ValueError(f"command id {button} is not allowed")
        query = {"button": int(button)}
        if value is not None:
            if button != BT_STR_S or not -100 <= int(value) <= 100:
                raise ValueError(f"invalid value {value!r} for command id {button}")
            query["value"] = int(value)
        return parse_settings(self.get("/command", query))
