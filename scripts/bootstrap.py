#!/usr/bin/env python3
"""Initialize missing submodules while preserving existing developer checkouts."""
import configparser
from pathlib import Path
import subprocess


def git(workspace, repository, *args, trust=(), capture=False, check=True):
    command = ["git"]
    # Bind-mounted repositories belong to the host user, not Docker's ROS user.
    for path in dict.fromkeys((workspace, repository, *trust)):
        command += ["-c", f"safe.directory={path}"]
    return subprocess.run(command + ["-C", str(repository), *args], check=check,
                          text=True, capture_output=capture).stdout


def initialize_submodules(workspace, repository):
    modules = configparser.ConfigParser(interpolation=None)
    modules.read(repository / ".gitmodules")
    for section in modules.sections():
        relative = Path(modules[section]["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"Submodule path must stay inside its parent: {relative}")
        target = repository / relative
        if not (target / ".git").exists():
            if target.exists() and (not target.is_dir() or any(target.iterdir())):
                raise SystemExit(f"Refusing to overwrite non-Git contents in {target}")
            # Update only this missing checkout. A recursive update would also
            # reset initialized children to the commits recorded by their parents.
            git(workspace, repository, "submodule", "update", "--init", "--checkout",
                "--", str(relative), trust=(target,))
        if target == workspace / "src/external/xarm_ros2":
            sparse = git(workspace, target, "config", "--bool", "--get", "core.sparseCheckout",
                         capture=True, check=False)
            if sparse.strip() != "true":
                # Only the manufacturer's description package is used by this demo.
                # Git preserves modified files outside the sparse selection.
                git(workspace, target, "sparse-checkout", "set", "--cone", "xarm_description")
        initialize_submodules(workspace, target)
        revision = git(workspace, target, "rev-parse", "--short", "HEAD", capture=True).strip()
        branch = git(workspace, target, "rev-parse", "--abbrev-ref", "HEAD", capture=True).strip()
        print(f"Keeping {target.relative_to(workspace)} at {revision} ({branch})", flush=True)


def bootstrap(workspace=None):
    workspace = Path(workspace).resolve() if workspace is not None else Path(__file__).resolve().parents[1]
    initialize_submodules(workspace, workspace)


if __name__ == "__main__":
    bootstrap()
