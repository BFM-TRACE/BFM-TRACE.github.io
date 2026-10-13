#!/usr/bin/env python3
"""TRACE's simulation-only HTTP bridge to the existing SONIC-H2 deployment.

Run with SONIC-H2's .venv_sim/bin/python. No robot network parameter exists:
both simulator and policy are fixed to loopback and a dedicated DDS domain.
"""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import pty
import queue
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class LiveSimulation:
    def __init__(self, root: Path, runtime: Path, domain: int):
        self.root, self.runtime, self.domain = root, runtime, domain
        self.lock = threading.RLock()
        self.commands = queue.Queue(maxsize=64)
        self.stop = threading.Event()
        self.process = None
        self.sim = None
        self.master = None
        self.status = 'starting'
        self.error = None
        self.mode = 'tracking'
        self.resume_mode = 'tracking'
        self.motion = 'squat_001__A359'
        self.velocity = (0., 0., 0.)
        self.planner_speed = .2
        self.last_command = time.monotonic()
        self.snapshot = {}
        self.fallen = False
        self.paused = False
        self.log_lines = deque(maxlen=120)
        reference = root / 'gear_sonic_deploy/reference/example_h2/deploy'
        manifest = json.loads((reference / 'h2_lora_reference_manifest.json').read_text())
        self.motions = [m['motion_name'] for m in manifest['motions']]
        self.reference = reference
        self.runtime.mkdir(parents=True, exist_ok=True)

    def state(self):
        with self.lock:
            return dict(self.snapshot, status=self.status, error=self.error,
                        mode=self.mode, motion=self.motion, motions=self.motions,
                        source='live_mujoco_policy', engine='MuJoCo + SONIC-H2 policy + SONIC planner',
                        policy='gpu3_20260907_stage2_step009450',
                        command=dict(vx=self.velocity[0], vy=self.velocity[1], yaw=self.velocity[2]),
                        body_pose_layout='xyz_wxyz', simulation_only=True,
                        dds_domain=self.domain)

    def submit(self, command):
        if not isinstance(command, dict):
            raise ValueError('JSON object required')
        if command.get('action') not in (None, 'reset', 'stop', 'play', 'start', 'pause'):
            raise ValueError('Unknown action')
        if command.get('mode') not in (None, 'tracking', 'planner'):
            raise ValueError('mode must be tracking or planner')
        if 'motion' in command and command['motion'] not in self.motions:
            raise ValueError('Unknown motion')
        for key in ('vx', 'vy', 'yaw'):
            if key in command:
                value = float(command[key])
                if not math.isfinite(value) or abs(value) > 1:
                    raise ValueError(f'{key} must be finite and between -1 and 1')
                command[key] = value
        # Movement freshness is measured when the browser request arrives,
        # including requests received while models are loading.
        self.commands.put_nowait(dict(command, _received_at=time.monotonic()))

    def send_keys(self, keys):
        if self.master is not None and self.process and self.process.poll() is None:
            os.write(self.master, keys.encode())

    def launch_policy(self):
        model = self.root / 'sonic_h2_native_release/experiments/gpu3_20260907_stage2_step009450'
        deploy = self.root / 'gear_sonic_deploy'
        env = {k: v for k, v in os.environ.items() if not k.startswith('GEAR_SONIC_')}
        env.pop('CYCLONEDDS_URI', None)
        env.update(GEAR_SONIC_DOMAIN_ID=str(self.domain), GEAR_SONIC_SKIP_MOTION_SWITCHER='1',
                   GEAR_SONIC_INITIAL_MOTION_NAME=self.motion,
                   GEAR_SONIC_AUTO_START_AFTER_SEC='8',
                   OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        command = [str(deploy / 'target/release/g1_deploy_onnx_ref'), 'lo',
                   str(model / 'model_decoder.onnx'), str(self.reference),
                   '--robot-profile', 'h2_native', '--encoder-file', str(model / 'model_encoder_full.onnx'),
                   '--obs-config', str(deploy / 'policy/release/observation_config_h2_native.yaml'),
                   '--planner-file', str(deploy / 'planner/target_vel/V2/planner_sonic.onnx'),
                   '--disable-crc-check', '--input-type', 'keyboard', '--output-type', 'zmq', '--zmq-out-port', '18877',
                   '--hand-backend', 'none']
        for gain in ('waist-pitch-kp', 'waist-roll-kp', 'ankle-pitch-kp', 'ankle-roll-kp', 'ankle-pitch-kd'):
            command.extend(['--h2-' + gain + '-scale', '1.0'])
        self.master, slave = pty.openpty()
        log = (self.runtime / 'policy.log').open('w')
        self.process = subprocess.Popen(command, cwd=deploy, env=env, stdin=slave,
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        os.close(slave)
        log.close()
        (self.runtime / 'policy.pid').write_text(str(self.process.pid))
        (self.runtime / 'launch.json').write_text(json.dumps(dict(command=command, domain=self.domain,
              simulation_only=True, interface='lo'), indent=2))
        deadline = time.monotonic() + 150
        while not self.stop.wait(.15):
            content = (self.runtime / 'policy.log').read_text(errors='replace')
            if 'Deploy controller ready' in content:
                return
            if self.process.poll() is not None:
                raise RuntimeError('Policy exited during initialization. See runtime/policy.log.')
            if time.monotonic() > deadline:
                raise RuntimeError('Policy initialization timed out. See runtime/policy.log.')

    def initialize_sim(self):
        sys.path.insert(0, str(self.root))
        from gear_sonic.utils.mujoco_sim.base_sim import BaseSimulator
        from gear_sonic.utils.mujoco_sim.configs import BaseConfig
        import numpy as np
        config = BaseConfig(wbc_version='h2_native_unitree_ros_loop', interface='lo',
                            enable_onscreen=False).load_wbc_yaml()
        config.update(DOMAIN_ID=self.domain, INTERFACE='lo', USE_JOYSTICK=False,
                      PRINT_SCENE_INFORMATION=False, REALTIME_MAX_LAG_MS=200)
        self.sim = BaseSimulator(config, onscreen=False)
        env = self.sim.sim_env
        original_step = env.sim_step
        last_snapshot = 0.

        def check_fall():
            if env.mj_data.qpos[2] < .28:
                self.fallen = True
                self.status = 'fallen'
        env.check_fall = check_fall

        def step():
            nonlocal last_snapshot
            with self.lock:
                if self.fallen:
                    # Keep the simulation's DDS heartbeat alive while freezing physics.
                    # The deployment watchdog must receive fresh LowState on resume.
                    env.unitree_bridge.PublishLowState(env.prepare_obs())
                    return
                original_step()
                if env.mj_data.time - last_snapshot >= 1 / 30 or not self.snapshot:
                    self.snapshot = dict(
                        sim_time=float(env.mj_data.time), timestamp=time.time(),
                        qpos=env.mj_data.qpos.tolist(),
                        body_poses=np.concatenate([env.mj_data.xpos, env.mj_data.xquat], axis=1).ravel().tolist(),
                        root_position=env.mj_data.qpos[:3].tolist(),
                        root_velocity=env.mj_data.qvel[:3].tolist(),
                        band_enabled=bool(env.elastic_band.enable),
                        body_names=[env.mj_model.body(i).name for i in range(env.mj_model.nbody)])
                    last_snapshot = float(env.mj_data.time)
        env.sim_step = step
        self.sim.start_as_thread()

    def reset(self):
        # Reload policy history as well as physics, rather than teleporting only the rendered robot.
        self.status = 'starting'
        self.resume_mode = self.mode
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.velocity = (0., 0., 0.)
        self.paused = False
        self.shutdown_policy()
        if self.sim:
            with self.lock:
                self.fallen = True
        self.mode = 'tracking'
        self.launch_policy()
        with self.lock:
            if self.sim:
                self.sim.sim_env.reset()
                self.sim.sim_env.elastic_band.enable = True
                self.snapshot = {}
            self.fallen = False
        return time.monotonic()

    def run(self):
        try:
            self.launch_policy()
            ready = time.monotonic()
            self.initialize_sim()
            self.status = 'warming_up'
            started = False
            last_keys = 0.
            while not self.stop.wait(.025):
                if self.process.poll() is not None:
                    raise RuntimeError('Policy process stopped; restart the simulation service.')
                if self.sim.sim_thread and not self.sim.sim_thread.is_alive():
                    raise RuntimeError('MuJoCo stopped (see server log for timing diagnostics).')
                if not started and time.monotonic() - ready >= 11:
                    if 'Init Done' not in (self.runtime / 'policy.log').read_text(errors='replace'):
                        continue
                    with self.lock:
                        self.sim.sim_env.elastic_band.enable = False
                    if self.resume_mode == 'planner':
                        self.send_keys('\n')
                        time.sleep(.35)
                        self.send_keys('1r')
                        self.mode = 'planner'
                        self.planner_speed = .2
                    else:
                        self.send_keys('t')
                    self.status = 'running'
                    started = True
                if not started:
                    continue
                try:
                    command = self.commands.get_nowait()
                except queue.Empty:
                    command = None
                if command:
                    self.last_command = command.pop('_received_at')
                    if time.monotonic() - self.last_command > 1 and command.get('mode') == 'planner':
                        command.update(vx=0., vy=0., yaw=0.)
                    if command.get('action') in ('pause', 'stop'):
                        self.paused = True
                        self.velocity = (0., 0., 0.)
                        self.send_keys('r')
                        self.status = 'paused'
                        continue
                    if command.get('action') == 'reset':
                        ready = self.reset()
                        self.status = 'warming_up'
                        started = False
                        continue
                    if self.fallen:
                        continue
                    if self.paused:
                        self.paused = False
                        self.status = 'running'
                    mode = command.get('mode', self.mode)
                    changed_mode = mode != self.mode
                    if mode != self.mode:
                        self.send_keys('\n')
                        self.mode = mode
                        self.velocity = (0., 0., 0.)
                        if mode == 'planner':
                            # Select SLOW_WALK (first standing motion-set entry).
                            time.sleep(.35)
                            self.send_keys('1')
                            self.planner_speed = .2
                    if mode == 'tracking':
                        selected = command.get('motion', self.motion)
                        count = (self.motions.index(selected) - self.motions.index(self.motion)) % len(self.motions)
                        for _ in range(count):
                            self.send_keys('n')
                            time.sleep(.07)
                        self.motion = selected
                        if changed_mode or 'motion' in command or command.get('action') == 'start':
                            self.send_keys('r')
                            time.sleep(.07)
                        self.send_keys('t')
                    else:
                        self.velocity = tuple(command.get(k, 0.) for k in ('vx', 'vy', 'yaw'))
                        speed = math.hypot(*self.velocity[:2])
                        if speed > .05:
                            speed = max(.2, min(.8, round(speed * 10) / 10))
                            increments = round((speed - self.planner_speed) * 10)
                            self.send_keys(('0' if increments > 0 else '9') * abs(increments))
                            self.planner_speed = speed
                    if command.get('action') == 'stop':
                        self.velocity = (0., 0., 0.)
                        self.send_keys('r')
                # Browser-held keys are renewed; expired commands become an actual planner stop.
                if self.mode == 'planner' and not self.fallen and not self.paused and time.monotonic() - last_keys > .12:
                    last_keys = time.monotonic()
                    if time.monotonic() - self.last_command > 1.0:
                        self.velocity = (0., 0., 0.)
                    vx, vy, yaw = self.velocity
                    keys = ''
                    if abs(vx) > .05:
                        keys += 'w' if vx > 0 else 's'
                    if abs(vy) > .05:
                        keys += ',' if vy > 0 else '.'
                    if abs(yaw) > .05:
                        keys += 'a' if yaw > 0 else 'd'
                    self.send_keys(keys or 'r')
        except Exception as exc:
            self.status, self.error = 'error', str(exc)
            import traceback
            traceback.print_exc()
        finally:
            # Reap the independently grouped policy even if simulator cleanup fails.
            try:
                self.shutdown_policy()
            finally:
                if self.sim:
                    self.sim.close()

    def shutdown_policy(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGCONT)
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        if self.master is not None:
            os.close(self.master)
            self.master = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sonic-root', required=True, type=Path)
    parser.add_argument('--runtime', required=True, type=Path)
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--domain', type=int, default=189)
    args = parser.parse_args()
    if args.domain < 180 or args.domain > 199:
        parser.error('Use a dedicated simulation DDS domain in 180..199')
    os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
    os.environ.pop('CYCLONEDDS_URI', None)
    os.chdir(args.sonic_root)
    simulation = LiveSimulation(args.sonic_root.resolve(), args.runtime.resolve(), args.domain)

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, document):
            data = json.dumps(document, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.split('?')[0] in ('/api/state', '/api/health'):
                self.reply(200, simulation.state())
            else:
                self.reply(404, dict(error='Not found'))

        def do_POST(self):
            if self.path != '/api/command':
                return self.reply(404, dict(error='Not found'))
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length < 1 or length > 8192:
                    raise ValueError('Body length must be 1..8192 bytes')
                simulation.submit(json.loads(self.rfile.read(length)))
                self.reply(202, dict(accepted=True, status=simulation.status))
            except (ValueError, TypeError, queue.Full) as exc:
                self.reply(400, dict(error=str(exc)))

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    worker = threading.Thread(target=simulation.run, daemon=True)
    worker.start()
    (args.runtime / 'server.pid').write_text(str(os.getpid()))
    print(f'TRACE simulation HTTP: http://127.0.0.1:{args.port} (DDS domain {args.domain}, lo only)', flush=True)

    def shutdown(signum, frame):
        simulation.stop.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        simulation.stop.set()
        worker.join(timeout=12)
        server.server_close()


if __name__ == '__main__':
    main()
