#!/usr/bin/env python3
"""Manage an independent, background SSH tunnel for the TRACE live simulator.

SSH handles authentication interactively or through the user's SSH agent.
Passwords are never accepted as arguments, stored, or added to subprocess env.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import socket
import subprocess
import sys
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('start', 'status', 'stop'))
    parser.add_argument('--host', '--target', dest='target', required=True,
                        metavar='USER@HOST', help='Explicit SSH destination; --target is a compatibility alias')
    parser.add_argument('--ssh-port', type=int, default=22)
    parser.add_argument('--local-port', type=int, default=8766)
    parser.add_argument('--remote-port', type=int, default=8766)
    parser.add_argument('--runtime', type=Path, default=Path(__file__).resolve().parents[2] / '.work/trace-homepage-tunnel')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+@[A-Za-z0-9_.-]+', args.target):
        parser.error('--host must be a user@host SSH destination')
    if any(port < 1 or port > 65535 for port in (args.ssh_port, args.local_port, args.remote_port)):
        parser.error('ports must be between 1 and 65535')
    runtime = args.runtime.resolve()
    runtime.mkdir(parents=True, exist_ok=True, mode=0o700)
    control = runtime / 'ssh.sock'
    if len(str(control).encode()) > 95:
        parser.error('runtime path is too long for an SSH control socket; choose a shorter --runtime')
    state_file = runtime / 'state.json'
    ssh_base = ['ssh', '-S', str(control), '-p', str(args.ssh_port)]

    def control_command(action):
        return subprocess.run(ssh_base + ['-O', action, args.target],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)

    def master_pid():
        if not control.exists():
            return None
        result = control_command('check')
        match = re.search(r'Master running \(pid=(\d+)\)', result.stdout + result.stderr)
        return int(match[1]) if result.returncode == 0 and match else None

    def report():
        pid = master_pid()
        result = dict(tunnel_running=bool(pid), pid=pid, target=args.target,
                      local_url=f'http://127.0.0.1:{args.local_port}',
                      control_socket=str(control), background_mode='ssh -fN, no idle expiry',
                      backend_reachable=False)
        if pid:
            process = subprocess.run(['ps', '-o', 'ppid=', '-p', str(pid)], capture_output=True, text=True)
            result['parent_pid'] = int(process.stdout.strip()) if process.stdout.strip() else None
            try:
                with urllib.request.urlopen(result['local_url'] + '/api/health', timeout=5) as response:
                    health = json.load(response)
                    result['backend_reachable'] = response.status == 200
                    result['backend'] = {key: health.get(key) for key in
                        ('status', 'source', 'mode', 'motion', 'sim_time', 'simulation_only', 'dds_domain', 'error')}
            except Exception as exc:
                result['backend_error'] = str(exc)
        return result

    if args.action == 'status':
        result = report()
        print(json.dumps(result, indent=2))
        return 0 if result['tunnel_running'] and result['backend_reachable'] else 1

    if args.action == 'stop':
        pid = master_pid()
        if pid:
            result = control_command('exit')
            if result.returncode:
                print(json.dumps(dict(stopped=False, error=result.stderr.strip())))
                return 1
            for _ in range(50):
                if master_pid() is None:
                    break
                time.sleep(.1)
        stopped = master_pid() is None
        if stopped:
            control.unlink(missing_ok=True)
            state_file.unlink(missing_ok=True)
        print(json.dumps(dict(stopped=stopped, pid=pid,
                              remote_service_unchanged=True)))
        return 0 if stopped else 1

    if master_pid():
        result = report()
        result['already_running'] = True
        print(json.dumps(result, indent=2))
        return 0 if result['backend_reachable'] else 1

    # Refuse to duplicate or replace a listener owned by another process.
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', args.local_port)) == 0:
            print(json.dumps(dict(started=False, error=f'Local port {args.local_port} is already in use; inspect its owner.')))
            return 1
    control.unlink(missing_ok=True)
    # -f performs SSH's own daemonization after authentication; the resulting
    # process is reparented to the OS, with stdin detached. ControlPersist=no
    # prevents a configured timed control-master lifecycle from taking over.
    command = ssh_base + [
        '-M', '-fNT', '-o', 'ControlMaster=yes', '-o', 'ControlPersist=no',
        '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=30',
        '-o', 'ServerAliveCountMax=3', '-o', 'ConnectTimeout=15',
        '-L', f'127.0.0.1:{args.local_port}:127.0.0.1:{args.remote_port}', args.target]
    result = subprocess.run(command)
    if result.returncode:
        return result.returncode
    for _ in range(50):
        if master_pid():
            break
        time.sleep(.1)
    result = report()
    result['started'] = result['tunnel_running']
    state_file.write_text(json.dumps(result, indent=2) + '\n')
    state_file.chmod(0o600)
    print(json.dumps(result, indent=2))
    return 0 if result['tunnel_running'] and result['backend_reachable'] else 1


if __name__ == '__main__':
    sys.exit(main())
