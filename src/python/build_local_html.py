#!/usr/bin/env python3
import argparse
import os
import re
import shutil
import subprocess
from pathlib import Path

from external.minify import html_minifier, rcssmin, rjsmin
from external.wheezy.template.engine import Engine
from external.wheezy.template.ext.core import CoreExtension
from external.wheezy.template.loader import FileLoader


def git_value(args, fallback):
    try:
        return subprocess.check_output(args, stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return fallback


def version_string(project_dir):
    version = (project_dir / "VERSION").read_text(encoding="utf-8").strip()
    sha = git_value(["git", "rev-parse", "--short=6", "HEAD"], "unknown")
    branch = git_value(["git", "rev-parse", "--abbrev-ref", "HEAD"], version)
    if version == "3.x.x":
        version = f"{version}-{branch}"
    return f"{version} ({sha})"


def render_file(engine, filename, context):
    template = engine.get_template(filename)
    data = template.render(context)
    if filename.endswith(".html"):
        data = html_minifier.html_minify(data)
    elif filename.endswith(".css"):
        data = rcssmin.cssmin(data)
    elif filename.endswith(".js"):
        data = rjsmin.jsmin(data)
    return data


def build_local_webui(project_dir, platform, output_dir):
    is_tx = "_TX" in platform.upper()
    is_8285 = "ESP8285" in platform.upper()
    has_sub_ghz = "_900_" in platform.upper() or "LR1121" in platform.upper()
    if "LR1121" in platform.upper():
        chip = "LR1121"
    elif has_sub_ghz:
        chip = "SX127X"
    else:
        chip = "SX128X"

    engine = Engine(
        loader=FileLoader([str(project_dir / "html")]),
        extensions=[CoreExtension("@@")]
    )
    context = {
        "VERSION": version_string(project_dir),
        "PLATFORM": re.sub("_via_.*", "", platform),
        "isTX": is_tx,
        "hasSubGHz": has_sub_ghz,
        "chip": chip,
        "is8285": is_8285,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    for filename in (
        "index.html",
        "scan.js",
        "mui.js",
        "elrs.css",
        "hardware.html",
        "hardware.js",
        "cw.html",
        "cw.js",
        "lr1121.html",
        "lr1121.js",
    ):
        (output_dir / filename).write_text(render_file(engine, filename, context), encoding="utf-8")

    readme = f"""ExpressLRS local Web UI for {context["PLATFORM"]}

Open index.html in a browser while the receiver WiFi is running.

The default API endpoint is http://10.0.0.1. To use a different device address,
open:

  index.html?host=192.168.1.50

The selected host is stored in browser local storage.
"""
    (output_dir / "README.txt").write_text(readme, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Build a local ExpressLRS Web UI package.")
    parser.add_argument(
        "--platform",
        default="Unified_ESP8285_2400_RX",
        help="Platform name without the _via_* suffix.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output directory. Defaults to local-webui/<platform>.",
    )
    args = parser.parse_args()

    project_dir = Path(__file__).resolve().parents[1]
    platform = re.sub("_via_.*", "", args.platform)
    output_dir = Path(args.output) if args.output else project_dir / "local-webui" / platform
    if output_dir.exists():
        shutil.rmtree(output_dir)
    build_local_webui(project_dir, platform, output_dir)
    print(output_dir)


if __name__ == "__main__":
    main()
