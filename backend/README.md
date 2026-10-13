# Legacy optional simulation backend

The current homepage runs policy inference and MuJoCo physics entirely in each browser Worker. **Normal startup needs neither this backend nor SSH**, and the current browser interface does not call this service. See [the homepage startup instructions](../README.md).

These tools preserve the earlier server-based simulation implementation for maintainers with a compatible SONIC-H2 deployment. The service runs the existing C++ policy, TensorRT planner and MuJoCo H2 simulator, and exposes simulated body transforms through HTTP. Starting it does not switch the current interface away from browser-local execution.

The backend is intentionally simulation-only: the policy and simulator use `lo`, DDS domain `189`, and no hand backend. HTTP binds to `127.0.0.1:8766`. Run one service per domain. The existing policy also publishes telemetry on ZMQ port `18877`; it does not accept browser commands on that port. No robot interface can be supplied to this service.

## Start on a configured SONIC-H2 server

Replace `user@simulation-host`, the SSH port, and `/path/to/SONIC-H2` below with your deployment settings. The tools contain no default project server or credentials. Copy `live_server.py` and `manage.py` into a separate directory on that server, for example `~/.cache/trace-homepage/backend`. No changes to SONIC-H2 source or model files are needed.

```sh
ssh -p 22 user@simulation-host 'mkdir -p ~/.cache/trace-homepage/backend'
scp -P 22 homepage/backend/live_server.py homepage/backend/manage.py \
  user@simulation-host:.cache/trace-homepage/backend/
```

Then, on the server:

```sh
python3 ~/.cache/trace-homepage/backend/manage.py start \
  --sonic-root /path/to/SONIC-H2
```

The manager uses SONIC-H2's existing `.venv_sim/bin/python`. Initialization loads three existing model engines, then starts the controller with the startup elastic band. It releases the band after the ready-pose initialization and starts the squat reference. Allow approximately 25 seconds. Logs and process IDs live in `~/.cache/trace-homepage/runtime`.

```sh
python3 ~/.cache/trace-homepage/backend/manage.py status \
  --sonic-root /path/to/SONIC-H2
python3 ~/.cache/trace-homepage/backend/manage.py stop \
  --sonic-root /path/to/SONIC-H2
```

Stop first, then start again to restart. `stop` signals only the PID recorded for this service and its policy child. It checks process identity before signalling.

Start the managed SSH tunnel on the computer running the homepage, from the project root:

```sh
python3 homepage/backend/tunnel.py start --host user@simulation-host --ssh-port 22
python3 homepage/backend/tunnel.py status --host user@simulation-host --ssh-port 22
```

Authenticate using your SSH agent/key or SSH's interactive password prompt. The script never accepts a password argument or writes credentials. It calls `ssh -fNT`, which authenticates first and then detaches into an independent background process, owned by the operating system instead of the terminal/tool session. `ControlPersist=no` prevents an inherited timed control-master setting from expiring the connection. The daemon has no idle expiry. `--host USER@HOST` is required for every command. `--ssh-port` defaults to 22; `--local-port` and `--remote-port` default to 8766. The previous `--target` flag remains an alias for `--host`. Use the same host, ports and runtime directory for start, status and stop.

The tunnel binds **only** `127.0.0.1:8766`. Its dedicated control socket and non-secret PID/status record live in `.work/trace-homepage-tunnel/`. `status` checks both the SSH daemon and the backend HTTP response; it reports the simulation source, mode, and time. Calling `start` again reuses a healthy managed tunnel and refuses to take over a port owned by another process. It never restarts the remote simulator.

The local Node server retains an optional `/api/` proxy to `http://127.0.0.1:8766` for legacy maintenance clients. The current demo does not use those routes. Keep this unauthenticated simulation API and its proxy on loopback; the static public website does not require either. To stop just the tunnel:

```sh
python3 homepage/backend/tunnel.py stop --host user@simulation-host --ssh-port 22
```

Closing a terminal or finishing an agent/tool call does not stop this daemon. Stop the remote simulator separately when finished. After a computer restart, or if a prolonged network outage causes the SSH keepalive to disconnect (30-second interval, three unanswered probes), rerun `start`. Password-based SSH cannot silently reauthenticate after a disconnect; the script uses your normal SSH authentication again. `status` reports a disconnected tunnel or unreachable backend explicitly.

For an existing managed connection, remote status/stop commands may reuse the same authentication by adding `-S "$PWD/.work/trace-homepage-tunnel/ssh.sock"` to `ssh`.

## HTTP protocol

- `GET /api/health` and `GET /api/state`: current state. `source` is `live_mujoco_policy`; `status` is `starting`, `warming_up`, `running`, `paused`, `fallen`, or `error`.
- `POST /api/command`: JSON; returns `202` after accepting the command. Commands are serialized on the simulation controller thread.
- Tracking: `{"mode":"tracking","motion":"squat_001__A359","action":"start"}`. The `motions` field supplies allowed exact IDs.
- Planner: `{"mode":"planner","vx":0.4,"vy":0,"yaw":0,"action":"start"}`. Forward/backward and lateral requests map to the deployment's keyboard directions. Their magnitude selects target speed in 0.1 m/s increments, clamped to the SLOW_WALK range of 0.2–0.8 m/s; zero requests stop. `yaw` is a directional turning input, not an angular velocity. These are planner targets, not measured velocities; `root_velocity` reports the simulation measurement. Renew held commands at least twice per second. After one second without a command, movement stops.
- `{"action":"pause"}` requests the controller's hold command. Planner movement stops; tracking returns to reference frame zero and holds that pose. Physics and posture feedback continue. `{"action":"play"}` restarts reference playback. This keeps the existing deployment's DDS watchdog active.
- `{"action":"reset"}` resets physics and reloads controller history, taking roughly 20 seconds, then restores the selected tracking/planner mode. A fallen robot is frozen and visibly reported as `fallen`; reset is explicit.

`body_poses` contains 45 × 7 flattened floats in MuJoCo body-ID order: `[x,y,z,qw,qx,qy,qz]`, matching the homepage H2 mesh asset. The state also contains `qpos`, `sim_time`, `root_position`, `root_velocity`, `body_names`, `band_enabled`, and model provenance. The snapshot rate is 30 Hz; browser polling at 10–20 Hz is sufficient. World coordinates are Z-up.

## Model and simulation provenance

- Encoder/decoder: `sonic_h2_native_release/experiments/gpu3_20260907_stage2_step009450/model_{encoder_full,decoder}.onnx` through the existing deployment executable and TensorRT engines.
- Planner: `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx` and its existing engine.
- Tracking references: `gear_sonic_deploy/reference/example_h2/deploy`, in manifest order, native H2 selector 0.
- Planner retains the deployment's G1 reference branch (selector 3); the optional G1→H2 comparison projection is not enabled.
- Simulator: `h2_native_unitree_ros_loop`, official `H2_loop.xml`, 1 ms physics, nominal Kp/Kd scales of 1.0, 50 Hz deployment policy.

This is an interactive demonstration of the current project checkpoint. It does not imply that every motion transition or planner input has been quantitatively validated. The backend reports falls and runtime errors instead of replacing them with synthetic animation.

## Historical validation and custom sessions

`qa.json` records the actual HTTP integration test: 315 body-pose values, controller hold/replay, planner forward displacement, command validation, and a complete physics/policy reset. It records observed values instead of claiming a motion-quality benchmark.

`tunnel-qa.json` records the connection-lifecycle regression: daemon parent PID 1 after the launching tool exited, idempotent start, confirmed stop, successful restart, and an advancing MuJoCo state through the homepage's own `/api/health` endpoint. The original remote simulation process was reused throughout.

The recorded reports describe historical sessions; their process IDs, host paths and observed states are provenance, not currently running service instructions. For a separate deployment session, choose your own runtime directory and pass it consistently:

```sh
ssh -p 22 user@simulation-host \
  'python3 ~/.cache/trace-homepage/backend/manage.py status --sonic-root /path/to/SONIC-H2 --runtime /path/to/trace-runtime'
ssh -p 22 user@simulation-host \
  'python3 ~/.cache/trace-homepage/backend/manage.py stop --sonic-root /path/to/SONIC-H2 --runtime /path/to/trace-runtime'
```

Only one instance should own port 8766 and DDS domain 189. The SSH tunnel is separate from the service; reconnecting it does not restart physics.
