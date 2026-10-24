"""Exercise initialization and preservation with disposable, local Git repositories."""
import contextlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from bootstrap import bootstrap


class BootstrapTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name).resolve()
        self.protocol = patch.dict(os.environ, {"GIT_ALLOW_PROTOCOL": "file"})
        self.protocol.start()
        self.addCleanup(self.protocol.stop)
        self.core = self.repository("core-origin")
        self.sim = self.repository("sim-origin")
        self.git(self.sim, "submodule", "add", str(self.core), "core")
        self.commit(self.sim)
        self.workspace = self.repository("workspace")
        self.git(self.workspace, "submodule", "add", str(self.sim), "src/external/sim")
        self.commit(self.workspace)

    def git(self, repository, *args):
        result = subprocess.run(["git", "-C", str(repository), *args], check=True,
                                text=True, capture_output=True)
        return result.stdout.strip()

    def repository(self, name):
        path = self.root / name
        path.mkdir()
        self.git(path, "init", "-b", "main")
        self.identity(path)
        (path / "source.txt").write_text("original\n")
        self.commit(path)
        return path

    def identity(self, repository):
        self.git(repository, "config", "user.name", "Bootstrap test")
        self.git(repository, "config", "user.email", "bootstrap@example.invalid")

    def commit(self, repository):
        self.git(repository, "add", ".")
        self.git(repository, "commit", "-m", "Fixture change")

    def initialize(self, workspace=None):
        with contextlib.redirect_stdout(io.StringIO()):
            bootstrap(workspace or self.workspace)

    def test_fresh_clone_initializes_nested_submodules(self):
        checkout = self.root / "fresh"
        self.git(self.root, "clone", "--no-local", str(self.workspace), str(checkout))
        self.initialize(checkout)
        sim = checkout / "src/external/sim"
        self.assertTrue((sim / ".git").is_file())
        self.assertTrue((sim / "core/.git").is_file())
        self.assertEqual(self.git(sim / "core", "rev-parse", "HEAD"),
                         self.git(self.core, "rev-parse", "HEAD"))

    def test_existing_branches_commits_and_dirty_files_are_preserved(self):
        self.initialize()
        repositories = [self.workspace / "src/external/sim",
                        self.workspace / "src/external/sim/core"]
        before = []
        for index, repository in enumerate(repositories):
            self.identity(repository)
            self.git(repository, "switch", "-c", f"development-{index}")
            (repository / "source.txt").write_text("committed development\n")
            self.commit(repository)
            (repository / "source.txt").write_text("uncommitted development\n")
            (repository / "pending.txt").write_text("untracked work\n")
        for repository in repositories:
            before.append((self.git(repository, "rev-parse", "HEAD"),
                           self.git(repository, "branch", "--show-current"),
                           self.git(repository, "status", "--porcelain")))
        self.initialize()
        for repository, expected in zip(repositories, before):
            actual = (self.git(repository, "rev-parse", "HEAD"),
                      self.git(repository, "branch", "--show-current"),
                      self.git(repository, "status", "--porcelain"))
            self.assertEqual(actual, expected)
            self.assertEqual((repository / "source.txt").read_text(), "uncommitted development\n")
            self.assertEqual((repository / "pending.txt").read_text(), "untracked work\n")

    def test_non_git_contents_in_missing_submodule_are_preserved(self):
        core = self.workspace / "src/external/sim/core"
        core.mkdir(exist_ok=True)
        (core / "local-work.txt").write_text("keep this\n")
        with self.assertRaisesRegex(SystemExit, "Refusing to overwrite"):
            self.initialize()
        self.assertEqual((core / "local-work.txt").read_text(), "keep this\n")

    def test_xarm_sparse_selection_and_nested_sdk_initialization(self):
        xarm = self.repository("xarm-origin")
        for package in ["xarm_description", "xarm_api"]:
            (xarm / package).mkdir()
            (xarm / package / "package.xml").write_text("fixture\n")
        self.git(xarm, "submodule", "add", str(self.core), "xarm_sdk/cxx")
        self.commit(xarm)
        self.git(self.workspace, "submodule", "add", str(xarm), "src/external/xarm_ros2")
        self.commit(self.workspace)
        self.initialize()
        checkout = self.workspace / "src/external/xarm_ros2"
        self.assertTrue((checkout / "xarm_description/package.xml").exists())
        self.assertFalse((checkout / "xarm_api/package.xml").exists())
        self.assertTrue((checkout / "xarm_sdk/cxx/.git").is_file())


if __name__ == "__main__":
    unittest.main()
