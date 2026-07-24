"""
Tests for the served web interface.

Motivated by a real 500: `/sw.js` was routed to StaticFileHandler with a regex
that captured no groups, so every request raised

    TypeError: StaticFileHandler.get() missing 1 required positional argument

plot_data.html registers the service worker on load, so this fired on every
visit to /plot and silently disabled offline tile caching. Nothing in the
Python test suite touched it because nothing rendered a page.

What is checked here:

  * every route returns 200 with the right content type
  * every asset the templates reference actually resolves
  * the HTML parses and contains the element IDs the JavaScript looks up
  * the JavaScript is syntactically valid (via node, when available)
  * the WebSocket payload contract matches what the browser code reads
  * malformed and hostile requests do not produce 500s

The last point matters more than it looks. A 500 on the boat means a blank
dashboard with no indication of why, which is the worst possible failure for
a display you are relying on.
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

HMI_DIR = Path(__file__).resolve().parent.parent
TEMPLATES = HMI_DIR / "templates"
STATIC = HMI_DIR / "static"


# ---------------------------------------------------------------------------
# Server fixture (module-scoped: spawning is the expensive part)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def web_server():
    """
    A real server on a non-default port.

    The port is deliberately NOT 5000. compass.js used to hardcode
    ws://host:5000/ws, which meant the page worked only on the default port
    and silently connected to nothing anywhere else. Running the tests off
    the default port is what keeps that regression caught.
    """
    import socket

    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()

    fixture = HMI_DIR / "tests" / "fixtures" / "underway_fast.log"
    if not fixture.exists():
        pytest.skip("fixture missing")

    proc = subprocess.Popen(
        [
            sys.executable,
            "app_tui.py",
            "--no-tui",
            "--offline",
            "--replay",
            str(fixture),
            "--speed",
            "50",
            "--port",
            str(port),
        ],
        cwd=str(HMI_DIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    base = f"http://127.0.0.1:{port}"
    deadline = time.time() + 25
    while time.time() < deadline:
        if proc.poll() is not None:
            out = proc.stdout.read() if proc.stdout else ""
            pytest.fail(f"server exited early:\n{out[-3000:]}")
        try:
            with urllib.request.urlopen(base + "/health", timeout=1) as r:
                json.loads(r.read())
            break
        except (urllib.error.URLError, OSError, ValueError):
            time.sleep(0.3)
    else:
        proc.terminate()
        pytest.fail("server did not become ready")

    yield base, port, proc

    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def fetch(base, path, timeout=10):
    """GET a path. Returns (status, body_bytes, headers). Never raises on 4xx/5xx."""
    req = urllib.request.Request(base + path)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestRoutes:
    @pytest.mark.parametrize(
        "path,content_type",
        [
            ("/", "text/html"),
            ("/plot", "text/html"),
            ("/health", "application/json"),
            ("/sw.js", "javascript"),
        ],
    )
    def test_returns_200_with_content_type(self, web_server, path, content_type):
        base, _, _ = web_server
        status, body, headers = fetch(base, path)
        assert status == 200, f"{path} returned {status}: {body[:300]!r}"
        assert content_type in headers.get("Content-Type", "").lower()
        assert len(body) > 0

    def test_sw_js_does_not_500(self, web_server):
        """
        Regression for the reported error. The route was

            (r"/sw.js", StaticFileHandler, {"path": ..., "default_filename": "sw.js"})

        which captures no groups. StaticFileHandler.get() requires a `path`
        positional argument that comes from the regex, and default_filename
        does not supply it -- that option only applies to directory requests.
        Result: TypeError, 500, on every page load of /plot.
        """
        base, _, _ = web_server
        status, body, _ = fetch(base, "/sw.js")
        assert status == 200, f"/sw.js returned {status}: {body[:300]!r}"

    def test_sw_js_serves_the_real_file(self, web_server):
        base, _, _ = web_server
        _, body, _ = fetch(base, "/sw.js")
        assert body == (STATIC / "sw.js").read_bytes()

    def test_unknown_route_404_not_500(self, web_server):
        base, _, _ = web_server
        status, _, _ = fetch(base, "/no-such-page")
        assert status == 404

    def test_health_is_valid_json(self, web_server):
        base, _, _ = web_server
        _, body, _ = fetch(base, "/health")
        snap = json.loads(body)
        assert isinstance(snap, dict)
        assert "heading_goal" in snap


# ---------------------------------------------------------------------------
# Assets
# ---------------------------------------------------------------------------


def referenced_assets():
    """Every src=/href= path the templates point at."""
    paths = set()
    for tpl in TEMPLATES.glob("*.html"):
        text = tpl.read_text(errors="replace")
        for m in re.finditer(r'(?:src|href)="(/static/[^"]+)"', text):
            paths.add(m.group(1))
    return sorted(paths)


@pytest.mark.integration
class TestAssets:
    @pytest.mark.parametrize("asset", referenced_assets())
    def test_referenced_asset_resolves(self, web_server, asset):
        """
        A 404 on an asset is a silent failure in the browser: the page renders
        but the compass never draws, or the map is blank. Parametrized so the
        failure names the missing file.
        """
        base, _, _ = web_server
        status, body, _ = fetch(base, asset)
        assert status == 200, f"{asset} -> {status}"
        assert len(body) > 0, f"{asset} served empty"

    def test_asset_list_is_not_empty(self):
        """Guard against the regex silently matching nothing."""
        assert len(referenced_assets()) >= 5

    def test_compass_js_served(self, web_server):
        base, _, _ = web_server
        status, body, _ = fetch(base, "/static/js/compass.js")
        assert status == 200
        assert b"WebSocket" in body

    def test_directory_traversal_blocked(self, web_server):
        """../ in a static path must not escape the static root."""
        base, _, _ = web_server
        for probe in (
            "/static/../app_tui.py",
            "/static/../../etc/passwd",
            "/static/%2e%2e/app_tui.py",
        ):
            status, body, _ = fetch(base, probe)
            assert status != 200 or b"import" not in body, f"{probe} leaked a file"


# ---------------------------------------------------------------------------
# HTML structure
# ---------------------------------------------------------------------------


def soup_for(base, path):
    from bs4 import BeautifulSoup

    _, body, _ = fetch(base, path)
    return BeautifulSoup(body, "html.parser")


@pytest.mark.integration
class TestHtmlStructure:
    @pytest.mark.parametrize("path", ["/", "/plot"])
    def test_parses_as_html(self, web_server, path):
        base, _, _ = web_server
        soup = soup_for(base, path)
        assert soup.find("html") is not None or soup.find("body") is not None

    @pytest.mark.parametrize("path", ["/", "/plot"])
    def test_has_no_unrendered_template_syntax(self, web_server, path):
        """
        Tornado template markers left in the output mean a substitution
        failed. The page still renders, so this is invisible without a check.
        """
        base, _, _ = web_server
        _, body, _ = fetch(base, path)
        text = body.decode("utf-8", "replace")
        for marker in ("{{", "}}", "{%", "%}"):
            assert marker not in text, f"unrendered template marker {marker!r} in {path}"

    @pytest.mark.parametrize("path", ["/", "/plot"])
    def test_script_tags_balanced(self, web_server, path):
        base, _, _ = web_server
        _, body, _ = fetch(base, path)
        text = body.decode("utf-8", "replace").lower()
        assert text.count("<script") == text.count("</script>")

    def test_main_page_has_ids_the_js_looks_up(self, web_server):
        """
        compass.js does getElementById on a set of names. A renamed or removed
        element gives a null reference and a dead widget -- with no error
        unless you have the console open.
        """
        base, _, _ = web_server
        soup = soup_for(base, "/")
        html_ids = {el.get("id") for el in soup.find_all(id=True)}

        js = (STATIC / "js" / "compass.js").read_text(errors="replace")
        wanted = set(re.findall(r"getElementById\(\s*['\"]([^'\"]+)['\"]", js))

        missing = wanted - html_ids
        assert not missing, (
            f"compass.js looks up IDs that autopilot.html does not define: "
            f"{sorted(missing)}"
        )

    def test_main_page_loads_compass_js(self, web_server):
        base, _, _ = web_server
        soup = soup_for(base, "/")
        srcs = [s.get("src", "") for s in soup.find_all("script")]
        assert any("compass.js" in s for s in srcs)


# ---------------------------------------------------------------------------
# JavaScript validity
# ---------------------------------------------------------------------------


NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="node not installed")
class TestJavaScriptSyntax:
    """
    A syntax error in an inline script kills every handler in that block, and
    the page still renders. `node --check` catches it cheaply.
    """

    def test_compass_js_parses(self):
        out = subprocess.run(
            [NODE, "--check", str(STATIC / "js" / "compass.js")],
            capture_output=True,
            text=True,
        )
        assert out.returncode == 0, f"compass.js syntax error:\n{out.stderr}"

    def test_sw_js_parses(self):
        out = subprocess.run(
            [NODE, "--check", str(STATIC / "sw.js")], capture_output=True, text=True
        )
        assert out.returncode == 0, f"sw.js syntax error:\n{out.stderr}"

    @pytest.mark.parametrize("tpl", sorted(p.name for p in TEMPLATES.glob("*.html")))
    def test_inline_scripts_parse(self, tmp_path, tpl):
        from bs4 import BeautifulSoup

        soup = BeautifulSoup((TEMPLATES / tpl).read_text(errors="replace"), "html.parser")
        blocks = [
            s.string
            for s in soup.find_all("script")
            if not s.get("src") and s.string and s.string.strip()
        ]
        if not blocks:
            pytest.skip(f"{tpl} has no inline script")

        for i, code in enumerate(blocks):
            # Tornado template expressions are not valid JS; skip blocks that
            # contain them rather than reporting a false syntax error.
            if "{{" in code or "{%" in code:
                continue
            f = tmp_path / f"{tpl}.{i}.js"
            f.write_text(code)
            out = subprocess.run(
                [NODE, "--check", str(f)], capture_output=True, text=True
            )
            assert out.returncode == 0, (
                f"{tpl} inline script #{i} syntax error:\n{out.stderr}"
            )


# ---------------------------------------------------------------------------
# WebSocket URL construction
# ---------------------------------------------------------------------------


class TestWebSocketUrl:
    """
    compass.js hardcoded ws://host:5000/ws. Running the server on any other
    port left the browser connecting to nothing -- no error banner, just a
    dashboard that never updated. The failure is invisible without the
    console open, which makes it exactly the kind of thing to pin.
    """

    @staticmethod
    def _code_only(text: str) -> str:
        """
        Strip // and /* */ comments.

        Without this the check matches its own explanatory comment, which
        would make the test permanently red for the wrong reason.

        The line-comment pattern requires the // not be preceded by a colon,
        so protocol literals like "ws://" and "https://" survive -- stripping
        those would delete the very code being checked for.
        """
        text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
        text = re.sub(r"(?m)(?<!:)//.*$", "", text)
        return text

    def test_compass_js_does_not_hardcode_a_port(self):
        js = self._code_only((STATIC / "js" / "compass.js").read_text(errors="replace"))
        assert ":5000" not in js, (
            "compass.js hardcodes port 5000; it should derive the port from "
            "location.port so --port works"
        )

    def test_compass_js_uses_location_port(self):
        js = (STATIC / "js" / "compass.js").read_text(errors="replace")
        assert "location.port" in js or "location.host" in js

    def test_compass_js_handles_https(self):
        """A page served over HTTPS cannot open a ws:// socket."""
        js = (STATIC / "js" / "compass.js").read_text(errors="replace")
        assert "wss://" in js

    def test_plot_page_uses_location_host(self):
        html = self._code_only((TEMPLATES / "plot_data.html").read_text(errors="replace"))
        assert "location.host" in html
        assert ":5000" not in html

    @pytest.mark.integration
    def test_websocket_connects_on_non_default_port(self, web_server):
        """
        End-to-end proof: the fixture runs on a random port, so a successful
        connection means the URL is being derived rather than assumed.
        """
        import asyncio

        base, port, _ = web_server
        assert port != 5000, "fixture should not use the default port"

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )
            msg = loop.run_until_complete(ws.read_message())
            assert msg is not None
            json.loads(msg)
        finally:
            loop.close()


# ---------------------------------------------------------------------------
# Payload contract
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestPayloadContract:
    """
    The browser reads two different message shapes off one socket: raw decoded
    CAN dicts (guarded with `'key' in data`) and periodic state snapshots.
    Both must keep arriving, and both must be JSON-serializable.
    """

    def _collect(self, base, seconds=4.0):
        import asyncio

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        msgs = []
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )

            async def pump():
                deadline = time.time() + seconds
                while time.time() < deadline:
                    try:
                        m = await asyncio.wait_for(ws.read_message(), timeout=1.0)
                    except asyncio.TimeoutError:
                        continue
                    if m is None:
                        break
                    msgs.append(json.loads(m))

            loop.run_until_complete(pump())
        finally:
            loop.close()
        return msgs

    def test_messages_arrive(self, web_server):
        base, _, _ = web_server
        assert self._collect(base) != []

    def test_all_messages_are_json_objects(self, web_server):
        base, _, _ = web_server
        for m in self._collect(base):
            assert isinstance(m, dict)

    def test_snapshot_messages_present(self, web_server):
        base, _, _ = web_server
        msgs = self._collect(base)
        assert any("ts" in m and "link_up" in m for m in msgs), (
            "no periodic snapshot seen; the broadcast tick may have stopped"
        )

    def test_keys_the_browser_reads_are_produced(self, web_server):
        """
        compass.js guards on `'x' in data` for a specific set of keys. If the
        server stops emitting one, the corresponding widget silently freezes
        at its last value -- which on a heading display is worse than blank.
        """
        base, _, _ = web_server
        seen = set()
        for m in self._collect(base, seconds=6.0):
            seen.update(m.keys())

        js = (STATIC / "js" / "compass.js").read_text(errors="replace")
        guarded = set(re.findall(r"['\"](\w+)['\"]\s+in\s+data", js))

        # Some guards are for keys only present on timeout notifications or
        # on the plot page; require the core telemetry set.
        core = {"rpm", "heading_goal", "autopilot_engaged"} & guarded
        missing = core - seen
        assert not missing, f"server never emitted keys the browser reads: {missing}"

    def test_no_nan_or_infinity_in_payloads(self, web_server):
        """
        Python's json.dumps emits bare NaN and Infinity, which are invalid
        JSON. JSON.parse throws on them, and the browser's onmessage handler
        dies for that message -- and every subsequent one if it is not caught.
        """
        base, _, _ = web_server
        import asyncio

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        raw = []
        try:
            from tornado.websocket import websocket_connect

            ws = loop.run_until_complete(
                websocket_connect(base.replace("http://", "ws://") + "/ws")
            )

            async def pump():
                deadline = time.time() + 4.0
                while time.time() < deadline:
                    try:
                        m = await asyncio.wait_for(ws.read_message(), timeout=1.0)
                    except asyncio.TimeoutError:
                        continue
                    if m is None:
                        break
                    raw.append(m)

            loop.run_until_complete(pump())
        finally:
            loop.close()

        assert raw, "no messages captured"
        for m in raw:
            for token in ("NaN", "Infinity", "-Infinity"):
                assert token not in m, (
                    f"payload contains bare {token}, which JSON.parse rejects: "
                    f"{m[:200]}"
                )


# ---------------------------------------------------------------------------
# Hostile input
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestRequestRobustness:
    @pytest.mark.parametrize(
        "path",
        [
            "/?" + "a" * 3000,
            "/health?callback=<script>alert(1)</script>",
            "/plot?x=%00",
            "/%2e%2e%2f",
            "/static/",
            "/static/js/",
            "/sw.js/../app_tui.py",
        ],
    )
    def test_odd_requests_do_not_500(self, web_server, path):
        """
        Anything a phone browser, a scanner, or a mistyped URL might produce.
        A 4xx is fine; a 5xx means an unhandled exception in a handler.
        """
        base, _, proc = web_server
        try:
            status, _, _ = fetch(base, path)
        except Exception:
            pytest.skip(f"client could not encode {path!r}")
        assert status < 500, f"{path} produced {status}"
        assert proc.poll() is None, "server died"

    def test_post_to_get_only_route(self, web_server):
        base, _, _ = web_server
        req = urllib.request.Request(base + "/", data=b"x=1", method="POST")
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                status = r.status
        except urllib.error.HTTPError as e:
            status = e.code
        assert status < 500

    def test_server_survives_the_whole_suite(self, web_server):
        base, _, proc = web_server
        assert proc.poll() is None
        status, _, _ = fetch(base, "/health")
        assert status == 200
