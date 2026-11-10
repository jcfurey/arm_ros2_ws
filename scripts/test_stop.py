"""Exercise stop.sh with inert process trees in disposable workspaces."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest


class StopTests(unittest.TestCase):
    def test_stops_descendants_and_preserves_another_workspace(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory) / "this workspace"
            other = Path(directory) / "other workspace"
            (workspace / "scripts").mkdir(parents=True)
            other.mkdir()
            stopper = workspace / "scripts/stop.sh"
            shutil.copyfile(Path(__file__).with_name("stop.sh"), stopper)
            marker = workspace / "child.pid"
            # These command-line tokens exercise direct ros2-launch discovery;
            # the processes themselves only sleep and handle termination.
            args = ["ros2", "launch", "arm_bringup", "demo.launch.py"]
            program = '''
import signal, subprocess, sys, time
from pathlib import Path
child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
def stop(_signal, _frame):
    child.terminate()
    child.wait()
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
Path("child.pid").write_text(str(child.pid))
time.sleep(60)
'''
            ours = subprocess.Popen([sys.executable, "-c", program, *args], cwd=workspace)
            unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", *args], cwd=other)
            try:
                deadline = time.monotonic() + 5
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertTrue(marker.exists(), "Test process did not initialize")
                child = int(marker.read_text())
                result = subprocess.run(["bash", str(stopper)], cwd=directory, capture_output=True,
                                        text=True, timeout=5)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("Simulation stopped.", result.stdout)
                self.assertEqual(ours.wait(timeout=2), 0)
                self.assertFalse(Path(f"/proc/{child}").exists(), "Descendant was left running")
                self.assertIsNone(unrelated.poll(), "Another workspace was stopped")

                again = subprocess.run(["bash", str(stopper)], capture_output=True, text=True, timeout=5)
                self.assertEqual(again.returncode, 0, again.stderr)
                self.assertIn("No simulation is running", again.stdout)
                self.assertIsNone(unrelated.poll())
            finally:
                for process in (ours, unrelated):
                    if process.poll() is None:
                        process.terminate()
                    process.wait(timeout=3)


if __name__ == "__main__":
    unittest.main()
