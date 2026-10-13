# Model and physics chain audit

The browser matches the server's **currently running step9450 simulation**. Model and physical configuration mismatches were not found that explain the recurring Run forward bow. This audit made no production changes, sent no robot/DDS commands, and did not run a closed-loop model A/B test.

- The actual running process’s `--planner-file` resolves to `target_vel/V2/planner_sonic.onnx` (773,952,989 bytes), freshly hashed as `39b553e197f62f077975ba38512bc04781a3fc37c2af7c6756e04629f760edea`. It is the only ONNX under the deployed planner directory; no V3 model was present there.
- Native H2 encoder and decoder SHA-256 hashes exactly match the running server models. The compact G1 planner encoder retains the original full encoder selector 3 weights; 473 CPU cases were bit-exact, and a fresh 14-fixture WASM check had zero token error and finite decoder outputs.
- The encoder selectors, full-root-relative orientation, joint order, history layout, default pose, action scales, action clipping, and nominal PD gains match the current simulation. A fresh server model initialization without DDS exactly matched the browser's physical configuration fields, including the H2 closed-loop transmission settings.
- The physical-robot launcher's **configured** checkpoint is step70000, with different ankle/knee gains. A stale comment says step5000. No active real-robot process was observed. Newer checkpoint metadata is not evidence that replacing the browser policy will improve this problem; controlled A/B testing would be needed.
- The pre-fix Run forward reference (SHA-256 `c3a90df46b2fc4d4a27ad13cc763ff0ec6b7dca67b52c874fcfec31d541cd2ca`) repeats every 1.96 seconds and contains root pitch up to 42.02° and waist pitch up to 30.49°. This is direct evidence of periodic bending in the source target, not proof of sole physical causation. The reference generator and bank were being corrected after this snapshot.
- Browser finite reference loops, phase blending, startup history, and WASM execution differ from the server's continuously replanned trajectories and TensorRT execution. Weight/configuration parity does not establish identical rollouts.

[Machine-readable evidence](model-chain-audit.json) records hashes, contracts, comparison fields, timestamps, and limitations. Raw read-only capture and collector remain in the workspace under `.work/browser-port/`.

## Resolution verified in browser-local physics

The reference export and selection were corrected without swapping encoder/decoder weights. The new forward reference uses the deployment token mask, cursor-context timing and a single stride. A 60-second Run-2.5 test reduced the observed maximum forward torso pitch from the pre-fix approximately 60.08° to 8.39°; maximum full-cycle torso excursion was 5.80°. Multiple speed targets replace the narrow single-clip cadence scaling, with a separately verified low-speed calibration.

The final [quality report](planner-quality.json) passes 16 cases; [boundary checks](planner-boundaries.json) pass 20; [mode/direction checks](planner-bank.json) pass 15; and [combined inputs](planner-combinations.json) pass 6. Requested forward speeds cover Slow 0.1–0.8, Walk 0.8–2.5 and Run 2.5–4.0 m/s, with lower documented sideways/backward caps. These are finite simulation checks, not claims of exact velocity tracking or hardware validation. [The revision summary](planner-speed-fix-summary.json) ties these results to the delivered files.
