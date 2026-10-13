# Browser-local H2 controller

This iframe runs the original H2 policy and MuJoCo physics on the visitor's device. Each page owns a dedicated module Worker with independent models, controller history and simulation state. The normal demo sends no state or command to a simulation server and does not connect to physical hardware.

Run `node server.mjs` from the parent `homepage` directory and open **http://127.0.0.1:4173/demo/**. Copy the whole `homepage` directory to another computer, including `demo/local-assets` and `demo/vendor`. Node.js 20+ serves the files; no npm install, Python, CUDA, SSH, or separately installed MuJoCo is needed. Do not open `index.html` using `file://`.

## Execution modes

**Motion tracking** reads typed references. Native H2 targets use the checkpoint’s native H2 encoder; native G1 exploration targets use the matching compact G1 selector-3 encoder. Both feed the same H2 decoder. Every 20 ms of simulation time, observations are gathered from the current simulated robot, inference produces 29 policy actions, and PD control runs 20 physics steps of 1 ms each. Rendered body poses come from MuJoCo contact and closed-loop mechanism dynamics. Reference data is not assigned to the moving robot's state.

**Planner control** uses a lightweight local gait target generator with four styles: `idle`, `slow_walk`, `walk` and `run`. Its bank contains 29 reference entries across four styles, cardinal directions and speed knots. Four low-speed entries preserve a generated full stride at an audited slower cadence to avoid an H2 motion dead zone. Online commands combine these native G1 targets, which go through the compact G1 encoder and the same H2 decoder and physics feedback. The G1 encoder retains the original full model's selector-3 branch and token quantizer; raw G1 joints are not projected into H2 reference coordinates before encoding.

The 773,952,989-byte learned planner is **not bundled or executed in the browser**. The bank implements four selected styles rather than all 27 original planner modes, and it does not establish equivalent planning performance. W/S selects forward/backward targets, A/D selects actual lateral gait targets, and Q/E changes an independent facing command. Combined inputs blend direction targets. Requested speed and turn values are targets, while the displayed measured speed comes from physics; exact speed tracking is not guaranteed.

`local-planner.js` aligns gait phases, blends directions and styles, and smooths start/stop transitions. Joint velocity targets are derived from the same time-scaled and blended future position trajectory, including the blend motion. Each cycle's mean source yaw is removed before its natural yaw oscillation is bounded to ±0.08 rad so large turns in some sideways-running samples do not replace the requested facing direction. A bounded correction based on measured physical heading error adjusts the orientation target as well. These operations change reference targets only; they do not overwrite body poses or translate the simulated robot. Strict facing preservation remains subject to the current closed-loop report.

**Reference only** is an explicit viewing option. It renders exported forward-kinematic body poses at 25 fps with interpolation from references originally sampled at 50 Hz. Nine H2 motions provide this optional preview: squat, lunge, dance, Macarena, walking with a turn, standing long jump, high jump and left/right high kicks. Seven additional m21 G1 references provide physical tracking only; their reference-only button is disabled and these entries cannot be selected inside reference-only mode. The interface labels this mode as kinematic playback, suspends the simulation, and disables planner controls. Scrubbing and playback speed apply only to this mode. **Return to browser simulation** resumes a controller view; there is no automatic fallback to playback after a simulation failure.

The default exporter now uses the deployment's 9–11-token mask, a fixed seed, cursor-based 30 Hz context, original replan intervals and eight-frame crossfade before selecting a single stride. The former forced 16-token, prediction-tail workflow introduced internal bending and is no longer the default. The offline reconstruction assumes zero inference latency; it is not an exact reproduction of the threaded server. Candidate quality is established separately in the actual H2 policy and physics loop. Idle and the three backward/sideways Run entries retain previously validated historical references and their original 16-token/tail-context provenance; the new speed knots use the corrected generation workflow.

Speed commands cover Slow walk 0.1–0.8, Walk 0.8–2.5 and forward Run 2.5–4.0 m/s. Composed inputs containing sideways/backward movement are limited to 0.8, 1.6 and 2.5 respectively, preserving vector direction. Higher raw-model speed labels are not promises of validated H2 performance. Braking is physical and takes time; the quality protocol observes the four-second stop interval, whereas the original default-speed matrix retains its three-second interval.

The m21 `reference/example_g1_explore` additions retain the confirmed README numbering: **#1 prayer (single-knee kneel and rise), #2 dejection (double-knee kneel and rise), #4 light jump, #5 broad jump A359**. C++ logs use zero-based indices 0, 1, 3 and 4. `q29G1`/`qd29G1` remain in the source IsaacLab semantic order; they are never projected into H2 `q31`. The renderer still shows actual H2 physics, not the G1 source pose.

Each of these four entries configures `startupHoldSeconds: 0.5`. For 25 policy steps, every future reference slot is source frame zero and desired joint velocity is zero, while the real encoder, decoder, histories, PD and physics keep advancing. Then the complete original sequence runs at 1× without resetting state, heading or feedback history. The source arrays are unchanged. This implements the deployed controller’s pre-play hold semantics with a fixed browser duration; the server waits for human input and has no fixed 0.5-second contract. The browser still holds the final target after completion.

The original G1 demo bundle also supplies the new `M21 · Right high kick`, `M21 · Left high kick` and `M21 · Right-foot hops`. The actual m21 folder is `reference/example`; the requested `reference/example_g1` name was absent in the inspected checkout. These clips use the same G1 encoder path and 0.5-second preparation. The two original kicks last 3.28 seconds each, and the complete hopping sequence lasts 9.98 seconds. [Candidate comparison](../qa/g1-standard-selection.json) records physical contact/flight and source-foot timing; [installed checks](../qa/g1-standard.json) run the complete motions followed by two seconds of final hold. Source-foot shape correlations describe timing and normalized shape only, not exact cross-embodiment tracking error.

The selected right-foot hopping sequence keeps the lifted left foot off the ground during the measured hopping interval. All five source rhythm peaks have contact-free intervals in the rollout, but the first three are shallow and only the last two meet the strict flight clearance/duration criterion. The mirrored candidate produces four larger qualifying hops but briefly places the other foot down between hops, so it was not selected. [Per-peak diagnostics](../qa/g1-standard-hop-detail.json) preserve this distinction.

The exact A359 broad jump is startup-sensitive: holds of 0, 0.4, 0.6, 1, 1.8 and 2.2 seconds failed, while 0.5 and 2 seconds passed in isolated checks. Neither successful duration establishes a broad convergence interval. The shipped fixed 0.5-second protocol must be used for the recorded validation; this does not establish success for arbitrary initial histories. [Startup audit](../qa/g1-explore-playback-audit.json), [comparison](../qa/g1-explore-startup-ab.json) and [installed-reference QA](../qa/g1-explore.json) retain these boundaries.

## Controls and lifecycle

- Choose a reference under **Motion tracking**. m21 G1 entries first show **Preparing motion** during the fixed physical startup hold. A new selection starts a fresh rollout. At its end, the controller holds the final target; it does not jump back to the first frame.
- **Pause** freezes local physics and policy stepping. **Resume** continues the same state. **Reset / Restart local rollout** resets the robot, controller history and reference time.
- Under **Planner control**, select Idle, Slow walk, Walk or Run. W/S requests forward/backward motion, A/D requests lateral motion, and Q/E independently requests turning. Keyboard and pointer inputs compose, including diagonals and translation while turning. Releasing one control preserves other held inputs; releasing all controls, losing focus or hiding the tab clears the request. The Worker also expires stale requests. Idle disables movement input.
- **Turn rate command** sets the independent Q/E or turn-button magnitude in 0.1 rad/s steps: Slow walk / Walk 0.1–1.0 rad/s; Run 0.1–0.3 rad/s. Dragging while held updates the command immediately. Changing style preserves the selected magnitude within the new limit; Run clamps larger values to 0.3. Idle, loading, reference-only mode and pending mode acknowledgement disable the slider. Higher Run rates were excluded after actual high-speed physics failures.
- Mode and style panels wait for a fresh matching Worker acknowledgement. Switching between Motion tracking and Planner control resets the simulation into a new rollout. Changing a planner style blends gait targets while maintaining physical feedback; this is not a full online neural-planner mode transition.
- Hidden pages suspend simulation stepping, and leaving the page terminates its Worker. Opening another page creates another independent simulation.
- On initialization failure, the model stays still with an error and a retry option. A fallen robot requires Reset. No synthetic movement is inserted to hide a failed rollout.

Drag to orbit, scroll to zoom, and use camera reset/follow as needed. The ground's 0.5 m checker squares and 2 m grid remain aligned to the world. Simulation time, measured planar root speed and real-time factor are shown separately from requested commands. Slow devices advance simulation more slowly rather than accumulating a backlog of physics steps.

## Local dependencies and compatibility

The bundle contains these local dependencies:

| Component | Version / size | Role |
| --- | --- | --- |
| Three.js | 0.180.0 | WebGL 2 rendering |
| MuJoCo | 3.10.0, approximately 10 MB runtime | Original H2 loop model, contact and constrained dynamics |
| ONNX Runtime Web | 1.30.0, approximately 14 MB runtime | Single-threaded CPU WASM inference |
| H2 native encoder | 16,950,929 bytes | `obs_dict` float32 `[1,640]` → `encoded_tokens` float32 `[1,64]` |
| Compact G1 encoder | 16,932,124 bytes | Same tensor sizes; original full encoder selector 3 for planner targets |
| H2 decoder | 40,743,594 bytes | `obs_dict` float32 `[1,994]` → `action` float32 `[1,29]` |

The three model files total **74,626,647 bytes** and contain their weights internally. All use ONNX opset 13. The native H2 encoder is the existing compact branch validated against selector 0. The compact G1 encoder was extracted from the deployed full ONNX without modifying its ten weight tensors or token quantizer; 473 sampled random and real-reference inputs match selector 3 bit for bit. No new weight quantization or model training is used here. Source hashes, extraction provenance, numerical checks and test fixtures are in `local-assets/models/`.

A modern browser needs WebAssembly SIMD, module Workers and WebGL 2. Inference uses `wasm.numThreads = 1` and `wasm.proxy = false`, because the application already owns the Worker. It does not require cross-origin isolation or WebGPU. Serve `.wasm` as `application/wasm`, as the included Node server does. All resources are local; no CDN is contacted.

The compact bundle includes only the CPU WASM provider. `local-inference.js` rejects any other provider explicitly. `vendor/ort/provenance.json` records the matching 1.30.0 JavaScript and WASM files and their hashes. See the official [deployment guide](https://onnxruntime.ai/docs/tutorials/web/deploy.html) and [runtime flags](https://onnxruntime.ai/docs/tutorials/web/env-flags-and-session-options.html).

Earlier Chromium checks demonstrated local execution and independent pause, reset and motion controls in two browser instances served by a plain static HTTP server, with no simulation API requests. The new directional-bank matrix passes 15 bounded cases over 174 simulated seconds, covering all four styles, cardinal directions, stopping and transitions. Performance and stability results apply to their recorded source revisions and machines, not every browser or device. Allow extra time and memory for initial loading of the three model files (74.6 MB) and local runtimes. The active reference bank is 3.9 MB.

Current evidence is separated by scope:

- [m21 G1 exploration checks](../qa/g1-explore.json): all four original references using the production startup path, full motion plus two seconds of terminal hold; [browser observations](../qa/g1-explore-browser.json) verify rendering and interaction. [H2 regression](../qa/g1-explore-h2-regression.json) and [planner regression](../qa/g1-explore-planner-regression.json) cover the existing four dynamic actions and 15 planner cases on this revision.
- [Earlier turn slider and dynamic-motion update](../qa/turning-and-dynamic-summary.json): source hashes at that revision, 33 UI checks, 24 physical turning cases, four complete dynamic actions and [browser observations](../qa/turning-and-dynamic-browser.json).

- [Model-chain audit](../qa/model-chain-audit.md): running step9450/V2 hashes, observation layout and physical configuration parity; step70000 belongs to the separate configured hardware launcher.
- [Final revision summary](../qa/planner-speed-fix-summary.json): 16 quality cases including a 60-second forward run, 20 boundaries, 15 style/direction cases and 6 composed-input cases. All pass for the recorded revision. [Walk stopping](../qa/walk-stop-fix.json) records the phase-dependent failure and five-phase verification of the 0.4-second target blend.
- [Quality checks](../qa/planner-quality.json) and [directional boundary checks](../qa/planner-boundaries.json): posture, complete-cycle oscillation, actual motion, speed response and braking at supported limits. Protocols and limitations are explicit in each report.
- [Compaction parity](../qa/planner-compaction-parity.json): exact future-target equality after unused-frame removal; [compactor](../scripts/compact-planner-bank.py) retains source hashes.
- [Encoder extraction and equivalence](local-assets/models/encoder-g1-validation.json): 473 cases against the full selector-3 model, including all 13 reference clips and perturbed base orientations. [WASM verification](local-inference-g1-wasm-qa.json) matches 14 stored fixtures using the shipped CPU runtime and also checks that the original tracking branch still runs.
- [Planner controls](../qa/planner-controls.json): 44 passing DOM event-harness cases for composed input, releases, independent turning, mode acknowledgements, typed reference routing and preparation/pause/restart timing. This does not exercise a real browser or physics.
- [Planner-bank closed-loop QA](../qa/planner-bank.json): actual simulated displacement, facing, stopping and style transitions. All 15 bounded cases pass: requested displacement dominates the orthogonal axis, heading drift stays below 0.3 rad and final-second stop RMS stays below 0.25 m/s. Refer to the recorded source hashes and protocol for the exact scope. [The pre-fix report](../qa/planner-bank-before-facing-fix.json) preserves the earlier heading failures.
- [Combined planner inputs](../qa/planner-combinations.json): all 6 bounded diagonal/translation-plus-turn physics cases pass, including stable stopping.
- [Turn-rate checks](../qa/planner-turn-rates.json): physical turning in both directions across the slider endpoints and midpoint, followed by a six-second stop observation. The [four-second report](../qa/planner-turn-rates-stop4s.json) preserves a short heading oscillation when Walk 2.5 m/s / −0.1 rad/s stops; isolated measurements show it settles in seconds 5–6. This is not a four-second settling guarantee. The existing 0.4-second Walk reference blend is unchanged; a 0.5-second candidate was rejected after a different stopping phase regressed.
- [Dynamic tracking checks](../qa/dynamic-motions.json): the four new motions complete the full reference and a two-second hold from the unchanged production reset; contact, foot clearance, kick reach, torso posture and terminal stability come from actual physics.
- [Browser interaction check](../qa/planner-browser.json): actual keyboard controls, idle/run selection and tracking/planner switching, with a saved screenshot and no captured console warnings or errors.

From `homepage`, `node demo/check_g1_encoder.mjs`, `node scripts/check-planner-controls.mjs` and `node scripts/check-planner-bank.mjs` regenerate these separate checks. `npm run check:physics` checks the physics implementation; older reports under `../qa/` retain their own scope and source revisions. These development checks do not replace the paper's evaluation.

## Controller and state contract

`local-policy.js` ports the H2-native deployment observation contract:

- Encoders: ten future samples, five reference frames apart at 50 Hz; 290 joint positions, 290 joint velocities and 60 root-relative orientation values. Native H2 tracking uses H2 joint order and `encode()`; G1 tracking and planner targets use raw G1 semantic joint order and `encodePlanner()`. The compact G1 branch retains the full encoder's internal 10×58 feature reshape and per-frame orientation concatenation.
- Orientation: wxyz quaternions, `sonic_full_root_relative_v0`, using the full inverse simulated root orientation. The six-dimensional representation is the first two rotation-matrix columns in row-major order.
- Decoder: 64 encoded tokens followed by ten oldest-first frames of angular velocity (30), joint position minus default (290), joint velocity (290), last action (290), and projected gravity (30).
- Actions: 29 outputs are clipped using the deployment limit, mapped into H2 local joint order, scaled, and added to default joint angles. PD torques and the H2 linkage transmission drive the MuJoCo model.

`local-worker.js` owns the loop and sends current physics state to `local-client.js`. State includes `source: 'browser_mujoco_policy'`, a per-page session ID, timestamp, simulation time, controller status, mode, motion, body poses, measured root velocity, inference timing and real-time factor. Body poses follow the rendered model's body order with `[x,y,z,qw,qx,qy,qz]` in the MuJoCo z-up world frame. Fresh finite state is required before the view enables controls.

The main modules are:

| File | Responsibility |
| --- | --- |
| `local-client.js` | Per-page Worker lifecycle and request/reply transport |
| `local-worker.js` | Initialization, command queue and control-loop scheduling |
| `local-policy.js` | Reference sampling, startup hold, observations, encoder selection and H2 action mapping |
| `local-reference.js` | Typed H2/G1 target validation and frame preparation without joint-space conversion |
| `local-planner.js` | Direction/style references, phase alignment, transitions and independent facing targets |
| `local-inference.js` | ONNX sessions and tensor validation |
| `local-physics.js` | MuJoCo loading, PD torques and closed-loop transmission mapping |
| `demo.js` | Three.js rendering, controls and state display |

The legacy server backend and its `/api/*` schema are documented in [../backend/README.md](../backend/README.md). They are retained for maintenance and are not used by this browser-local interface.

## Assets and provenance

`local-assets/controller.json` records policy ordering and constants. `local-assets/physics/` contains the original exported H2 loop model, meshes and runtime configuration. `local-assets/references/` contains typed H2 and G1 tracking targets. The reference manifest selects the encoder space and optional startup hold; individual G1 JSON files record original m21 source hashes and numbering. [The planner-bank manifest](local-assets/planner-bank/manifest.json) records the active native G1 direction/style/speed samples and derived low-speed cadence, their offline generation settings and source hashes. These are control targets, not recorded simulated body poses. Model and runtime manifests record source paths, versions and hashes; source paths are provenance only and are not runtime dependencies.

The four dynamic additions are unmodified native H2 retargeted CSV sequences: `broad_jump_003__A360`, `high_jump_R_opt_2_001__A477`, `neutral_kick_trash_R_003__A098` and its original library mirror `_M`. The kick sequences include approach/departure steps. Their named 31-joint targets are converted from 120 Hz cm/degrees to 50 Hz m/radians, with root quaternion SLERP, following the original H2 motion-library converter. They use the existing native selector-0 encoder and step9450 decoder with the normal default reset, gains and contact model. No special initial pose, manual body animation or new checkpoint is used.

`export_dynamic_motions.py` and `dynamic-motion-sources.json` reproduce these targets and the separate FK previews in the source SONIC-H2 Python/MuJoCo environment. The exporter writes manifests: **always give it a new isolated output directory**, never the installed `local-assets` or `assets` directories. Original CSV paths are provenance only; they are not runtime dependencies. Native JSON records source/converter hashes; the viewer manifest distinguishes original CSV `sourceSHA256`, exported target `referenceSHA256` and FK `previewSHA256`.

```sh
python export_dynamic_motions.py --root /path/to/SONIC-H2 \
  --spec dynamic-motion-sources.json --output /path/to/new-isolated-candidates
```

Update the spec source paths for another source checkout. Run `npm run check:dynamic` from `homepage` to verify installed targets. Candidate sequences that fall after landing or fail to achieve actual flight are excluded; successful FK playback alone does not qualify.

`assets/` contains the separate reference viewer's visual meshes and forward-kinematic pose files. To regenerate those preview assets, use the SONIC-H2 MuJoCo environment and an explicit output directory:

```sh
.venv_sim/bin/python export_assets.py --root /path/to/SONIC-H2 --output /path/to/assets
```

This export reads the original model and native references without starting inference, DDS or robot interfaces. It leaves auxiliary passive joints at default values for the reference preview; browser simulation instead solves the actual closed-loop mechanism. `assets/motions.json` records source hashes and sampling rates; `assets/h2-model.json` records the XML hash.

`export_g1_encoder.py` reproduces selector-3 branch extraction and comparison against the original full ONNX; `check_g1_encoder.mjs` verifies the shipped model and stored expected outputs with local WASM. The extraction report includes the source model and script hashes.

To generate new **offline planner candidates**, run `export_planner_bank.py` in a Python environment containing NumPy, SciPy and ONNX Runtime, with access to the original SONIC-H2 checkout:

```sh
python export_planner_bank.py --root /path/to/SONIC-H2 \
  --output /path/to/isolated-candidates --mode 3 --direction forward \
  --speeds 2.5 4 --seed 1234 --seconds 12
```

Moving modes are 1 (slow walk), 2 (walk) and 3 (run). Directions are forward, back, left, right, forward_left and forward_right; movement is independent of a fixed forward-facing target. The exporter uses the deployment's 9–11 prediction tokens, a fixed seed across replans, current-cursor context, two-frame lookahead and eight-frame blend, under a zero-inference-latency assumption. It selects one same-leg stride after warmup and preserves genuine learned positions and orientations. It writes continuous NPY data, generation provenance, candidate JSON when source screening passes, and `planner-candidate-report.json`, with SHA-256 hashes. `--rescore` reuses and verifies its saved continuous data; `--compare-tail` is an explicit historical A/B diagnostic only. See `--help` for source-screening options and the optional `--ort-site` dependency path.

Source screening and requested speed are **not H2 physical validation**. Test each candidate's actual motion, posture, direction, transitions and stopping before installing it. Low-speed reference motion can produce almost no actual H2 displacement. The exporter never updates the active manifest and rejects an output directory containing `manifest.json`.

Verification files in this directory state their test scope. Earlier `live-first-qa.json` and server QA records describe the legacy connection-based implementation; they are not evidence for browser-local physics. Browser inference and physics have separate benchmark and parity records. `local-inference-wasm-smoke.json` checks the CPU-only runtime with the original two models and synthetic observations in Node. Earlier inference timings used a larger WebGPU-capable runtime on its WASM backend and are retained as historical measurements, not measurements of this final three-model architecture.

Three.js and ONNX Runtime Web use MIT licenses; MuJoCo uses Apache 2.0. Their licenses and ONNX Runtime's third-party notices are bundled under `vendor/`. Robot assets, references and checkpoint weights retain their source project's usage terms.

### Exporting the m21 G1 references

`export_g1_explore_candidates.py` converts the original `example_g1_explore` CSV bundle into raw 50 Hz G1 reference JSON; it requires only Python standard libraries. Run it into a new, isolated candidate directory, then select and physically validate the intended entries before copying them into the active manifest. The exporter also includes unselected exploration clips, so do not replace the active manifest wholesale. The installed four entries and source hashes are recorded in [g1-explore-installation.json](../qa/g1-explore-installation.json). No source CSV or remote mount is needed at runtime.

`export_g1_standard_candidates.py` performs the corresponding CSV-to-JSON export for the original G1 `reference/example` collection and writes foot-position measurements for offline QA. Installed tracking references are self-contained; the measurements under `qa/g1-standard-source-measurements/` are only needed for reproducing the quality checks.
