# Browser inference assets

These are real source assets from the current SONIC-H2 checkpoint. Browser
simulation uses the native encoder and decoder to compute control actions,
then advances the closed-loop H2 model with MuJoCo. The motion JSON files are
**reference targets** for the policy; they must not be assigned to the rendered
robot as if they were simulation output.

- `models/encoder_native.onnx`: 16,950,929 bytes, float32, opset 13; `obs_dict`
  `[1,640]` → `encoded_tokens` `[1,64]`.
- `models/decoder.onnx`: 40,743,594 bytes, float32, opset 13; `obs_dict`
  `[1,994]` → `action` `[1,29]`.
- `models/manifest.json`: exact source paths, checksums, tensor signatures,
  operator counts, and metadata. The large full encoder and original planner
  are documented there but **are not bundled**.
- `controller.json`: policy/local joint order, nominal angles, action scales,
  gains, history layout, model interfaces, and projected initial configuration.
- `physics/model.xml` and `physics/meshes/`: the official closed-loop H2 model
  after the same runtime configuration used by the existing simulator, saved
  with MuJoCo 3.10.0. The model has 68 qpos, 61 qvel, 31 actuators and 45 bodies.
- `physics/physics-config.json`: joint addresses, actuator order, transmission
  groups, actuator limits, and exact initial qpos/qvel. Local-to-simulation
  joint order is identity for this profile. Motor-response and ankle-response
  additions are disabled in the baseline model.
- `physics/transmission-regression.json`: two configurations, each with all
  five closed-loop Jacobians and a deterministic torque-mapping example. The
  Python baseline reloads the exact portable XML so comparisons do not mix the
  pre-save and post-save floating-point geometry.
- `references/manifest.json`: native H2 references at 50 Hz, including the
  existing default stand and a generated slow-walk reference.

The native encoder is the model's H2 branch, not a substitute trained for this
website. The source's validation records in `models/source-validation.json`
establish numerical equivalence with selector 0 of the larger full encoder.
The original planner has 773,952,989 bytes of ONNX data and is intentionally
excluded from this minimal browser package.

## Active planner assets

Planner mode uses `models/encoder_g1.onnx` (16,932,124 bytes), an exact extraction of full encoder selector 3, followed by the same native H2 decoder. `planner-bank/manifest.json` selects 29 active direction/style/speed entries. Browser commands use the limits in this manifest; legacy planner fields in `controller.json` are not the active reference selection.

The multi-speed scheduler phase-aligns strides, derives consistent qdot from future q, removes each cycle's global yaw offset, and retains real policy/contact dynamics. The bank includes explicitly recorded low-speed cadence calibration. It is approximately 3.9 MB after exact loop-frame slicing; `compaction.json` records source and output hashes. See the parent demo README for regeneration and closed-loop quality reports.

## Historical slow-walk reference provenance

The following initial export is retained for source history. It is **not selected by the active planner-bank manifest**.


`references/planner_walk_04.json` was generated **offline** from the original
learned planner on CPU, using target speed 0.4 m/s, forward direction and seeds
9450–9456. It contains the final single prediction after seven autoregressive
calls, so the exported motion has no concatenation boundary. The output was
resampled from 30 to 50 Hz with quaternion SLERP and projected using the exact
`G1PlannerReferenceToH2NativePolicy` mapping from the source. The two head
joints retain their zero defaults. No simulator or robot was controlled during
this extraction.

The full reference has 72 frames. Its recommended loop is `[1,68)`, 67 frames
or 1.34 seconds; average reference forward speed is about 0.3406 m/s. The loop
was selected by joint-position and joint-velocity endpoint similarity without
altering the generated frames. This is a candidate reference, **not evidence
that the resulting H2 policy rollout is stable**. The browser must test the
complete policy/physics loop. Runtime direction/phase planning around this
library is distinct from running the original large learned planner online.

`planner_walk_04_raw_g1.npy` preserves the source prediction for inspection.
The JSON embeds the original model checksum, requested settings, timing, and
context-reconstruction discrepancies. The planner reconstructs its context
approximately; it does not copy those frames bit-for-bit.

## Re-extraction

Use `../export_local_assets.py` inside SONIC-H2 with `.venv_sim/bin/python`,
passing `--root` and `--output`. It uses `DefaultEnv` directly and never creates
DDS, a robot connection, or a deployment controller. Copy the two ONNX models
named above separately and verify their recorded SHA-256 hashes.

Then run `../export_planner_reference.py` in a Python environment with NumPy,
SciPy and ONNX Runtime, passing the same `--root` and the exported
`references/` directory as `--output`. The optional `--ort-site` locates an
existing ONNX Runtime package without installing or modifying the source
environment. It updates the motion manifest when one exists.

All browser runtime files are self-contained. These extraction steps, Python,
SSH, and the source server are development tools, not browser dependencies.
