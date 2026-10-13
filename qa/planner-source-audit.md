# Planner source audit — 2026-10-10

Scope: read-only deployment review and isolated CPU ONNX reference generation. No DDS, simulator service, or robot process was started. Files under `source-audit-*` are candidates; exporting a reference does not validate H2 physical execution.

## Main finding

The old offline bank exporter forced 16 prediction tokens, advanced its autoregressive context to the last four frames of each full prediction, and changed the random seed on each invocation. That differs from the deployment. The active run-forward loop also covered three strides. Its raw root pitch ranged from −2.81° to 42.02° and raw waist pitch from 9.23° to 30.49°. The largest bow occurred inside the loop, not at its wrap. Root/waist had a 0.51 Hz component while the left knee repeated at 1.53 Hz: one bow per three strides. All four old forward seed candidates shared the problem.

The 30-to-50 Hz conversion and G1 semantic joint order were checked separately and were correct. Across 13 original bank clips, the export matched the C++ conversion to maximum joint-position error 2.89e−15 and velocity error 1.44e−13. This is not a 30/50 Hz speed-unit mistake.

## Deployment facts

Paths below are relative to `/home/qingyul/workspace/SONIC-H2`.

- `gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/localmotion_kplanner.hpp:98`: documented ranges are slow walk 0.1–0.8 m/s, walk 0.8–2.5 m/s, run 2.5–7.5 m/s. These describe G1 planner input ranges, not validated H2 execution limits.
- The same header:233–235 uses two control-frame lookahead and initial random seed 1234. `UpdatePlanning` near line 575 places the new sequence at the current generation frame plus that lookahead; lines 643–693 sample four context frames at 30 Hz.
- `include/localmotion_kplanner_tensorrt.hpp:96–102` enables 9, 10, or 11 prediction tokens, producing 36, 40, or 44 frames. The stale nearby comment claims a different mask; the executed assignments are authoritative. `UpdateInputTensors` near line 375 keeps the previous seed when called with −1.
- `src/g1_deploy_onnx_ref.cpp:242–245` requests periodic run replans every 0.1 s and ordinary walk replans every 1 s. `CurrentFrameAdvancement`, near lines 4289–4405, blends a new plan over eight control frames. The subsequent context comes from the current reference buffer, not the tail of the full unused prediction.
- `docs/source/references/planner_onnx.md`: positive `target_vel` is a desired speed in m/s; a nonpositive value selects the mode's default speed and does not mean stop. Motion is shaped by a critically damped spring and learned motion dynamics, so achieved reference speed need not equal that target. Outputs are 30 Hz.
- The Pico/VR sender has a separate speed limiter (`gear_sonic/scripts/pico_manager_thread_server.py:591–625`). Direct ONNX generation does not pass through this limiter.

## Controlled evidence

Same mode 3 and target 2.5 m/s, with the old tail-context/changing-seed recurrence retained as a controlled ablation:

| Token mask | Raw mean forward speed | Root pitch range | Waist pitch range |
|---|---:|---:|---:|
| Forced 16 tokens | 1.433 m/s | −2.61…44.68° | 16.89…30.07° |
| Deployment 9–11 tokens | 2.134 m/s | 2.30…12.48° | 14.48…23.25° |

This isolates the excessive forced prediction horizon as a major cause of the bowing and speed suppression. Full deployment-timing reconstruction further changes the motion, so horizon is not claimed to be the only difference.

`planner-source-audit.py` reconstructs fixed-seed, deployment-token-mask inference; four-frame context at 30 Hz; the two-frame lookahead; the eight-frame crossfade; and mode-specific replan intervals. It assumes **zero inference latency**, rather than emulating asynchronous TensorRT scheduling. It creates 12 seconds of continuous genuine model output, discards the first four seconds for cycle selection, and selects adjacent same-leg knee-swing peaks. No root orientation or joint angle is clamped.

For run target 2.5 and fixed seed 1234, the selected loop is frames 304–334 (30 frames, one stride). Its reference forward speed is 3.220 m/s, root pitch −11.09…−6.21°, and waist pitch 3.26…15.63°. Parent-agent browser-policy/MuJoCo validation subsequently measured approximately 2.98 m/s actual H2 forward speed, maximum torso pitch 8.62°, and stable stopping. These source targets remain distinct from measured physical speeds.

The original loop also has a secondary seam problem: cyclic seam velocity RMS 3.298 rad/s versus natural 2.229 rad/s; seam derivative discrepancy RMS 1.919 rad/s. Improving the seam alone cannot remove the interior bow.

## Current candidate workflow

The isolated investigation initially kept the original exporter unchanged. The shipped `demo/export_planner_bank.py` now incorporates the corrected version-3 workflow; historical scripts and candidate outputs remain under the workspace’s `.work/browser-port/`. During the investigation, `planner-source-audit-v2.py` added an explicit `stable-lean` candidate screen for high-speed running. It reports pitch peak-to-peak and extrema; it permits constant source lean while still rejecting large within-stride bowing. This only chooses unmodified model references for subsequent tests and does not change physical-test thresholds.

Candidates must still pass actual H2 closed-loop motion, stopping, direction, and torso-quality tests before installation. The documented planner ceiling of 7.5 m/s must not be advertised as an H2 capability unless those tests establish it. Reference speed, requested target speed, and measured physical speed must be reported separately.

The first high-speed screen exposed another cycle-selection problem: a minimum knee-peak separation of 24 frames suppresses alternate peaks when the true stride becomes 20 frames at target 7.5. Version 3 lowers the run peak-distance minimum to 16 frames, then selects adjacent detected same-leg peaks. All high-speed candidates were rescored from the saved continuous trajectories, without rerunning or altering the learned outputs. The resulting 7.5 candidates have 20-frame single strides, rather than the incorrectly labeled 40-frame two-stride loops. Version 3 also offers `--rescore` to perform this extraction without ONNX inference.

Four fixed seeds (0, 42, 19450, and 21450) were tested independently at targets 3.5, 4, 5.5, and 7.5. Target 4 has candidates with small root-pitch oscillation and good joint seams. Target 5.5 introduces sustained source root pitch around 10–20° plus waist pitch around 20–30°. Target 7.5 consistently introduces much stronger sustained root pitch around 27–45° plus waist pitch around 24–29°, despite modest within-stride pitch oscillation. That differs from the old intermittent bow, but does not establish comfortable or stable H2 execution. The most periodic target-7.5 source (seed 19450) has a 20-frame stride, root pitch 27.41–34.17°, waist 23.85–27.14°, and q-seam MSE 3.1e-07. Its torso lean still merits rejection if the physical demo cannot execute it naturally.

`planner-source-audit-candidates.json` indexes all downloaded forward candidates, their hashes, source speeds, posture ranges, and one-stride metadata. No candidate was installed by this audit. For reproduction, run the script with `--root`, `--ort-site`, an isolated `--output`, `--mode 3 --direction forward --speeds 3.5 4 5.5 7.5 --seconds 12 --seed 42 --posture-mode stable-lean`. Each speed takes approximately 5–8 seconds of CPU inference on the source server. Use `--rescore` only when that directory already contains the matching continuous NPY files and preserve the same generating seed in the metadata.

Two intermediate targets were then examined at seeds 42 and 19450. The target-4.5/seed-19450 candidate is especially useful for physical evaluation: frames 541–566, reference forward speed 4.786 m/s, root pitch −2.14…3.84°, waist pitch 10.04…18.46°, and seam-velocity discrepancy RMS 0.867 rad/s. Target-5/seed-42 gives frames 481–506, reference forward speed 5.143 m/s, root pitch 3.12…7.08°, waist pitch 20.70…26.44°, and seam discrepancy RMS 0.237 rad/s. The target-5/seed-19450 trajectory failed the unchanged yaw screening gate. These intermediate candidates provide a way to evaluate a genuine motion-speed boundary rather than time-warping the good target-2.5 clip.

Evidence files: `planner-source-audit-local.json`, `planner-source-audit-ablation.json`, `planner-source-audit-run-speeds.json`, and per-seed `source-audit-*/planner-source-audit.json`. Candidate provenance includes model and generator SHA-256, fixed seed, timing assumption, and selected one-stride interval.

## Installed result and execution limits

The active manifest selects one-stride forward Run targets 2.5 (seed 1234) and 4.0 m/s (seed 42), plus independently validated backward/sideways references. Requested speed is not measured H2 speed: the final forward sweep measured approximately 2.97, 3.30, 3.59 and 3.86 m/s for Run requests 2.5, 3.0, 3.5 and 4.0. A 60-second Run-2.5 rollout measured maximum forward torso pitch 8.39° and full-cycle excursion 5.80°, compared with approximately 60.08° and 48.59° in the pre-fix physical rollout. Consult `planner-quality.json` for the exact source revision, protocol and complete signed extrema.

Candidate labels above 4.0 were not exposed. The 4.5/seed-19450 candidate fell after approximately 12.16 seconds; 5.0 and 5.5 candidates failed the unchanged four-second braking check. The initial 4.0/seed-1235 candidate also failed a stop-phase posture check and was replaced by the seed-42 candidate. These outcomes reject the candidates for this demo rather than establish a universal controller speed limit.

Low-amplitude 0.1 m/s source motion produced almost zero actual H2 progress, so the installed low knot uses an unmodified complete 0.4 m/s stride at half cadence. Native 0.4 and 0.8 targets form the other slow-walk knots. Derived entries retain explicit tempo-calibration provenance; the resulting forward sweep is checked for real movement and monotonic speed response.

Each loop’s mean yaw is removed before retaining a bounded natural yaw oscillation; this prevents a fixed sampled heading offset from steering the robot away from the commanded facing. Joint positions, root roll and root pitch are not clamped to hide the bow. Unused frames are removed only for packaging, with exact future-position/velocity/quaternion parity checked separately.
