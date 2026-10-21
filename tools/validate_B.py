#!/usr/bin/python3
"""Own and audit an isolated full ten-task simulation, with safe cleanup."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import re
import subprocess
import time
import psutil

ROOT=Path(__file__).resolve().parents[1]


def source_manifest():
    skip={'.venv','work','build','install','log','generated','metrics','__pycache__','.pytest_cache'}
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in ROOT.rglob('*') if p.is_file() and not any(x in skip for x in p.relative_to(ROOT).parts)}


def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['cbs','eecbs'],required=True)
    p.add_argument('--run',required=True);p.add_argument('--domain',type=int,default=44);args=p.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.run):raise SystemExit('Run name must use letters, digits, dash or underscore')
    out=ROOT/'work/validation_B'/args.run
    if out.exists():raise SystemExit('Refusing to overwrite previous run')
    out.mkdir(parents=True);metrics=ROOT/'metrics'/args.run
    cfg=ROOT/'src/hetero_fleet_scheduler/config'/('channels_cbs.yaml' if args.mode=='cbs' else 'channels.yaml')
    manifest=source_manifest();(out/'source-manifest.json').write_text(json.dumps(manifest,indent=2))
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION='huaxue304_B_'+args.run)
    log=(out/'launch.log').open('w')
    cmd=['ros2','launch','hetero_fleet_bringup','bringup.launch.py','headless:=true','rviz:=false',
         'channels_file:='+str(cfg),'metrics_dir:='+str(metrics)]
    launch=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);log.close()
    (out/'process.json').write_text(json.dumps(dict(launch_pid=launch.pid,mode=args.mode,domain=args.domain,gz_partition=env['GZ_PARTITION'],command=cmd),indent=2))
    print('LAUNCH',launch.pid,out,flush=True)
    children=[]
    try:
        with (out/'audit.log').open('w') as audit:
            result=subprocess.run(['/usr/bin/python3',str(ROOT/'tools/validate_full_batch.py'),
                '--scenario',str(ROOT/'src/hetero_fleet_scheduler/config/tasks/demo_10.yaml'),
                '--metrics-dir',str(metrics),'--output',str(out),'--launch-pid',str(launch.pid),
                '--expected-mapf-mode',args.mode],cwd=ROOT,env=env,stdout=audit,stderr=subprocess.STDOUT)
        if result.returncode:raise RuntimeError('Full batch audit failed; inspect '+str(out/'failure.json'))
        after=source_manifest()
        if manifest!=after:raise RuntimeError('Source changed during validation')
        (out/'source-verified.json').write_text(json.dumps(dict(unchanged=True,files=len(after)),indent=2))
        print('PASSED',args.mode,flush=True)
    finally:
        try:children=psutil.Process(launch.pid).children(recursive=True)
        except psutil.Error:pass
        try:os.killpg(launch.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:launch.wait(timeout=12)
        except subprocess.TimeoutExpired:pass
        for child in reversed(children):
            try:
                if child.is_running():child.terminate()
            except psutil.Error:pass
        _,alive=psutil.wait_procs(children,timeout=3)
        for child in alive:
            try:child.kill()
            except psutil.Error:pass
        if launch.poll() is None:launch.kill();launch.wait()
        (out/'cleanup.json').write_text(json.dumps(dict(owned_launch_stopped=launch.poll() is not None,
                                                        descendants_recorded=len(children)),indent=2))


if __name__=='__main__':main()
