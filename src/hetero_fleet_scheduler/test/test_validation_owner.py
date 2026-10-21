import importlib.util
from pathlib import Path
import subprocess
import sys
import time

script=Path(__file__).resolve().parents[3]/'tools/validate_full_batch.py'
spec=importlib.util.spec_from_file_location('batch_audit',script);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_dead_unreaped_launch_is_not_a_continuous_simulation():
    process=subprocess.Popen([sys.executable,'-c','pass'])
    path=Path(f'/proc/{process.pid}/stat')
    try:
        start=path.read_text().split()[21]
        limit=time.monotonic()+3
        while time.monotonic()<limit and path.read_text().split()[2]!='Z':time.sleep(.01)
        assert path.exists() and not module.owner_running(path,start)
    finally:process.wait()


def test_process_identity_requires_same_live_launch():
    path=Path('/proc/self/stat');start=path.read_text().split()[21]
    assert module.owner_running(path,start)
    assert not module.owner_running(path,str(int(start)+1))
