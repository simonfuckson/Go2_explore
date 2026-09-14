#!/usr/bin/env python3
"""Exercise runner conflicts, Ctrl+C and snapshot validation using observe only."""
import fcntl,hashlib,json,os,signal,subprocess,tempfile,time,uuid
from pathlib import Path
from session import WS,LOCK,STATE,ticks,verify_snapshot

def main():
    report={}
    executable=str(WS/'run_go2_explore')
    with LOCK.open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rejected=subprocess.run([executable,'observe','lock_refusal_only'],text=True,capture_output=True)
        assert rejected.returncode!=0 and 'stack lock' in rejected.stderr
        assert not (WS/'maps/lock_refusal_only').exists()
        report['shared_lock_refusal']=True
    with tempfile.TemporaryDirectory(dir=str(WS/'artifacts')) as directory:
        path=Path(directory)
        names=('public_map.pcd','traversed_path_map.pcd')
        for name in names:(path/name).write_bytes(b'checksum fixture')
        (path/'mapping_snapshot.sha256').write_text(''.join(hashlib.sha256((path/n).read_bytes()).hexdigest()+'  '+n+'\n' for n in names))
        assert verify_snapshot(path)
        (path/names[1]).write_bytes(b'corrupted')
        assert not verify_snapshot(path)
        (path/'mapping_snapshot.sha256').unlink()
        assert not verify_snapshot(path)
        report['corrupt_or_missing_snapshot_rejected']=True
    name='ctrlc_static_'+uuid.uuid4().hex[:8]
    with (WS/'artifacts/ctrlc_console.log').open('wb') as output:
        child=subprocess.Popen([executable,'observe',name],stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+22
            while time.monotonic()<deadline:
                if child.poll() is not None:raise RuntimeError('observe exited before Ctrl+C')
                time.sleep(.2)
            session=json.loads(STATE.read_text())
            assert not session['real_sdk'] and not session['auto_start']
            status=subprocess.run([executable,'status'],text=True,capture_output=True,timeout=10)
            assert status.returncode==0 and '/terrain/status' in status.stdout
            (WS/'artifacts/ctrlc_status.txt').write_text(status.stdout)
            os.kill(child.pid,signal.SIGINT)
            child.wait(timeout=60)
            result=json.loads((Path(session['session_dir'])/'session_result.json').read_text())
            assert result['success'] and not result['forced_shutdown'],result
            assert not STATE.exists()
            report.update(ctrlc_clean_save=True,session=result,map_name=name,session_dir=session['session_dir'])
        finally:
            if child.poll() is None:
                os.kill(child.pid,signal.SIGINT);child.wait(timeout=60)
    rejected=subprocess.run([executable,'observe',name],text=True,capture_output=True)
    assert rejected.returncode!=0 and 'overwrite' in rejected.stderr
    report['existing_map_refused']=True
    with LOCK.open('a') as lock:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    report['lock_released']=True
    (WS/'artifacts/session_lifecycle.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
