#!/usr/bin/env python3
"""Fetch pinned robot and Ouster simulation sources, preserving local work."""
import json
from pathlib import Path
import subprocess


def bootstrap_xarm():
    workspace = Path(__file__).resolve().parents[1]
    dependency = json.loads((workspace / "dependencies/xarm.json").read_text())
    target = workspace / "src/external/xarm_ros2"

    def git(*args, capture=False):
        return subprocess.run(
            ["git", "-c", f"safe.directory={target}", "-C", str(target), *args],
            check=True, text=True, capture_output=capture,
        ).stdout

    if target.exists() and not (target / ".git").is_dir():
        raise SystemExit(f"Refusing to overwrite existing non-Git directory: {target}")
    target.mkdir(parents=True, exist_ok=True)
    if not (target / ".git").exists():
        git("init", "--quiet")
        git("remote", "add", "origin", dependency["url"])
    else:
        head = subprocess.run(
            ["git", "-c", f"safe.directory={target}", "-C", str(target), "rev-parse", "--verify", "HEAD"],
            text=True, capture_output=True,
        )
        if head.returncode == 0:
            if head.stdout.strip() != dependency["revision"]:
                raise SystemExit(f"{target} is at a different revision; move it aside before bootstrapping.")
            if git("status", "--porcelain", capture=True).strip():
                raise SystemExit(f"{target} has local changes; preserve them before bootstrapping.")
            if (target / "xarm_description/package.xml").is_file():
                print(f"xarm_description is pinned at {dependency['revision'][:12]}")
                return

    git("sparse-checkout", "init", "--cone")
    git("sparse-checkout", "set", *dependency["paths"])
    git("fetch", "--depth=1", "--filter=blob:none", "origin", dependency["revision"])
    git("checkout", "--detach", dependency["revision"])
    print(f"Fetched xarm_description at {dependency['revision'][:12]}")


def bootstrap_ouster():
    workspace = Path(__file__).resolve().parents[1]
    dependencies = json.loads((workspace / "dependencies/ouster.json").read_text())
    for dependency in dependencies["repositories"]:
        target = workspace / "src/external" / dependency["directory"]

        def git(*args, check=True, capture=False):
            return subprocess.run(
                ["git", "-c", f"safe.directory={target}", "-C", str(target), *args],
                check=check, text=True, capture_output=capture,
            )

        if target.exists() and not (target / ".git").is_dir():
            raise SystemExit(f"Refusing to overwrite {target}")
        target.mkdir(parents=True, exist_ok=True)
        if not (target / ".git").exists():
            git("init", "--quiet")
            git("remote", "add", "origin", dependency["url"])
        head = git("rev-parse", "--verify", "HEAD", check=False, capture=True)
        if head.returncode == 0:
            if head.stdout.strip() != dependency["revision"]:
                raise SystemExit(f"{target} differs from dependencies/ouster.json; preserve your checkout and update the pin intentionally.")
            # Existing developer edits are allowed; never reset a source checkout.
            submodules = git("submodule", "status", "--recursive", capture=True).stdout
            if any(line.startswith("-") for line in submodules.splitlines()):
                raise SystemExit(f"Initialize missing submodules with git -C {target} submodule update --init --recursive")
            print(f"{dependency['directory']} is pinned at {dependency['revision'][:12]}")
            continue
        git("fetch", "--depth=1", "--filter=blob:none", "origin", dependency["revision"])
        git("checkout", "--detach", dependency["revision"])
        git("submodule", "update", "--init", "--recursive")


if __name__ == "__main__":
    bootstrap_xarm()
    bootstrap_ouster()
