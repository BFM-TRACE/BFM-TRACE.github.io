#!/usr/bin/env python3
"""Start/stop the isolated HTTP simulator on the SONIC-H2 host."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    parser.add_argument('--sonic-root', required=True, type=Path)
    parser.add_argument('--runtime', type=Path, default=Path.home() / '.cache/trace-homepage/runtime')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--domain', type=int, default=189)
    args = parser.parse_args()
    runtime = args.runtime.resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    pidfile = runtime / 'server.pid'
    pid = int(pidfile.read_text()) if pidfile.exists() else None
    alive = False
    if pid:
        try:
            # Never signal an unrelated process after PID reuse.
            cmdline = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
            alive = 'live_server.py' in cmdline and str(runtime) in cmdline
        except (FileNotFoundError, ProcessLookupError):
            pass
    if args.action == 'stop':
        if alive:
            os.kill(pid, signal.SIGTERM)
            for _ in range(200):
                try:
                    stat = Path(f'/proc/{pid}/stat').read_text()
                    if stat[stat.rfind(')') + 2:].split()[0] == 'Z':
                        alive = False
                        break
                except FileNotFoundError:
                    alive = False
                    break
                time.sleep(.1)
        print(json.dumps(dict(stopped=not alive, pid=pid,
                              error='Service did not stop within 20 seconds' if alive else None)))
        if alive:
            raise SystemExit(1)
        return
    if args.action == 'start':
        if alive:
            print(json.dumps(dict(already_running=True, pid=pid)))
            return
        command = [str(args.sonic_root / '.venv_sim/bin/python'),
                   str(Path(__file__).with_name('live_server.py').resolve()),
                   '--sonic-root', str(args.sonic_root.resolve()),
                   '--runtime', str(runtime), '--port', str(args.port), '--domain', str(args.domain)]
        with (runtime / 'server.log').open('w') as log:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        pidfile.write_text(str(process.pid))
        print(json.dumps(dict(started=True, pid=process.pid, url=f'http://127.0.0.1:{args.port}',
                              log=str(runtime / 'server.log'))))
        return
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{args.port}/api/health', timeout=3) as response:
            status = json.load(response)
            print(json.dumps({key: status.get(key) for key in
                  ('status', 'error', 'mode', 'motion', 'sim_time', 'source', 'policy')}))
    except Exception as exc:
        print(json.dumps(dict(status='offline', error=str(exc))))


if __name__ == '__main__':
    main()
