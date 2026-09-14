#!/usr/bin/env python3
"""Run the production exporter on copies, using a private localhost ROS master.

Accepts arbitrary saved map directories. Never loads navigation or chassis nodes.
Source files, including existing PGM and terrain products, are verified unchanged.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import time
import xmlrpc.client
import yaml


def hashes(root):
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.iterdir() if p.is_file()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-map', type=Path, action='append', required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--exporter', type=Path, required=True)
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1]
    args.output_root.mkdir(parents=True, exist_ok=False)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_LOG_DIR=str(args.output_root/'roslogs'))
    os.environ.pop('ROS_HOSTNAME', None)
    results = []
    with (args.output_root/'master.log').open('w') as master_log:
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        try:
            for attempt in range(50):
                try:
                    if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/offline_test')[0] == 1:
                        break
                except (OSError, xmlrpc.client.Error):
                    pass
                time.sleep(.2)
            else:
                raise RuntimeError('Private test master failed to start')
            for source in args.source_map:
                source = source.resolve()
                before = hashes(source)
                output = args.output_root/source.name
                output.mkdir()
                try:
                    for name in ('public_map.pcd', 'traversed_path_map.pcd',
                                 'mapping_snapshot.sha256', 'map.yaml', 'map.pgm'):
                        shutil.copy2(source/name, output/name)
                    params = yaml.safe_load((package/'config/terrain_export.yaml').read_text())
                    export_id = 'regression-' + source.name
                    params.update(map_dir=str(output), export_id=export_id)
                    config = output/'export_parameters.yaml'
                    config.write_text(yaml.safe_dump(params, sort_keys=False))
                    subprocess.run(['rosparam', 'delete', '/go2_terrain_exporter'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    subprocess.run(['rosparam', 'load', str(config), '/go2_terrain_exporter'], check=True)
                    begin = time.monotonic()
                    with (output/'export.log').open('w') as log:
                        process = subprocess.run([str(args.exporter)], stdout=log,
                                                 stderr=subprocess.STDOUT, timeout=300)
                    record = dict(map=source.name, seconds=round(time.monotonic()-begin, 2),
                                  exit_code=process.returncode)
                    if process.returncode == 0:
                        subprocess.run(['/usr/bin/python3', str(package/'scripts/validate_terrain_map.py'),
                                        '--map-dir', str(output), '--expected-export-id', export_id], check=True)
                        record['quality'] = yaml.safe_load((output/'terrain_quality.yaml').read_text())
                        # A bad source snapshot must not replace even one committed asset.
                        committed = hashes(output)
                        with (output/'public_map.pcd').open('ab') as stream:
                            stream.write(b'\ninvalid snapshot test\n')
                        with (output/'failure_test.log').open('w') as log:
                            failed = subprocess.run([str(args.exporter)], stdout=log, stderr=subprocess.STDOUT, timeout=30)
                        assert failed.returncode != 0, 'Corrupted snapshot unexpectedly exported'
                        shutil.copy2(source/'public_map.pcd', output/'public_map.pcd')
                        after = hashes(output)
                        assert all(after[k] == v for k, v in committed.items()), 'Failed export modified committed assets'
                        record['failed_export_preserved_assets'] = True
                    else:
                        record['error'] = (output/'export.log').read_text()[-2200:]
                    results.append(record)
                    print(json.dumps(record), flush=True)
                finally:
                    assert hashes(source) == before, 'Source map changed during regression'
        finally:
            os.killpg(master.pid, signal.SIGINT)
            try:
                master.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(master.pid, signal.SIGTERM)
                master.wait(timeout=5)
    (args.output_root/'results.json').write_text(json.dumps(results, indent=2))
    return int(any(r['exit_code'] for r in results))


if __name__ == '__main__':
    raise SystemExit(main())
