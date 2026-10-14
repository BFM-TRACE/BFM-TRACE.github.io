# Browser-local SONIC H2 demo

The interactive demo uses the released weights and all 38 references from M21's `~/workspace/SONIC_unitree_h2` (revision recorded in `local-assets/source-inventory.json`). Each visitor runs an independent Worker, ONNX Runtime WASM and MuJoCo WASM. Body poses come from physical simulation; reference data supplies control targets.

## Released models

- `local-assets/models/encoder_full.onnx`: exact released `model_encoder_full.onnx`, 66,317,960 bytes, float32 `[1,1751] -> [1,64]`.
- `local-assets/models/decoder.onnx`: exact released `model_decoder.onnx`, 40,743,596 bytes, float32 `[1,994] -> [1,29]`.
- `encoder_native.onnx` is the exact source compact H2 model, retained for source integrity checks; the browser uses the universal encoder.

No weights are retrained, quantized, merged or extracted. The universal encoder uses H2 selector 0, SMPL selector 2 and G1 selector 3. Robot observations sample ten futures 0.1 seconds apart; SMPL samples ten futures 0.02 seconds apart. SMPL world-axis joint vectors are rotated into their own reference root frame before encoding. Root-relative orientation, oldest-first history, joint order, action scaling and nominal gains follow the released runtime. The source licenses and model attribution are retained in `local-assets/models/SOURCE-LICENSE` and `SOURCE-NOTICE`.

## Motion controls

The menu contains all 8 H2, 13 basic G1, 7 exploration G1, 8 SMPL and 2 generated speech-gesture references. Collection/type labels distinguish different representations of the same motion. Speech gestures play motion only in the web demo. Source hashes and the original 50 Hz timeline are preserved.

Each reset holds the initial reference under live policy and physics for 0.5 seconds (1.0 second for the right-foot hopping clip), then runs the complete motion and holds the terminal target. Pause freezes physics and reference time; Resume preserves history; Restart initializes a new rollout. Source command-line replay instead releases its body channel on completion and returns to standing.

Only native H2 targets offer `View reference only`: this explicitly labeled mode uses separate FK preview binaries and does not execute policy or physics. G1 and SMPL use their own encoder inputs and never project their reference joints onto an H2 animation.

`Planner control` retains the 29-entry offline directional bank. The source `models/motion_planner_g1/planner_sonic.onnx` is byte-identical to its generator, as recorded in the bank manifest; new released policy weights and physics execute these targets. W/A/S/D compose translation; Q/E set independent turning. Releases, blur and hidden tabs stop requested motion. Speed and turn values are requests, not guaranteed measured rates. The approximately 774 MB neural planner is not run online.

## Verification and regeneration

- `node scripts/check-m21-parity.mjs`: 114 independent source replay/CPU ONNX fixtures covering every clip at start, middle and end, checked against browser observation assembly and actual WASM inference.
- `node scripts/check-m21-release.mjs`: every complete reference plus a two-second terminal hold, finite-state/fall checks and planner regression. These bounded checks do not establish tracking accuracy or physical-robot safety.
- `npm run check:controls`: UI pause/reset, typed routing, preparation and planner input checks.
- `npm run check:release`: source resources, controls, static build integrity and HTTP delivery.

Reports are in `qa/generated/m21-parity.json` and `qa/generated/m21-release.json`. Older checkpoint, dynamic-motion and compact-G1 audits are retained as historical records; they are not evidence for this release.

Run `demo/export_m21_release.py` with the source repository's `.venv_sim/bin/python`, `--root` and a new isolated `--output` directory. It initializes only `DefaultEnv` without DDS, copies exact ONNX bytes, verifies source motion hashes, exports the matching closed-chain model/configuration and H2 FK previews. `demo/export_m21_fixtures.py` independently samples the source Python replay contract for cross-runtime validation. Source paths are provenance; no remote filesystem or robot connection is used at runtime.
