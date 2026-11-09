#!/usr/bin/env python3
"""Package the dependency-free MoveIt panel for Lichtblick desktop or web."""
import argparse
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "lichtblick" / "moveit-panel"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", action="store_true", help="Also install into this user's Lichtblick desktop extensions directory.")
    args = parser.parse_args()
    package = json.loads((SOURCE / "package.json").read_text())
    output = ROOT / "lichtblick" / f"{package['publisher']}.{package['name']}-{package['version']}.foxe"
    files = {"package.json": (SOURCE / "package.json").read_bytes(),
             "dist/extension.js": (SOURCE / "src" / "extension.js").read_bytes(),
             "README.md": (SOURCE / "README.md").read_bytes()}
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, content in files.items():
            info = zipfile.ZipInfo(name, date_time=(2026, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, content)
    print(output)
    if args.install:
        target = Path.home() / ".lichtblick-suite" / "extensions" / f"{package['publisher']}.{package['name']}-{package['version']}"
        for name, content in files.items():
            path = target / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        print(f"Installed: {target}")
        print("Reload Lichtblick and import lichtblick/workcell.json to open the MoveIt tab.")


if __name__ == "__main__":
    main()
