# Released browser assets

The active model and reference manifests are exported from `QinnyuLan/SONIC_unitree_h2`, with its exact revision and source-file hashes in `source-inventory.json`.

`models/encoder_full.onnx` and `models/decoder.onnx` are unmodified released ONNX files. The browser loads these two models (107,061,556 bytes total). `models/encoder_native.onnx` preserves the source compact model for integrity checks. `models/manifest.json` records signatures, hashes and which models are bundled. `SOURCE-LICENSE` and `SOURCE-NOTICE` retain the source terms and attribution.

`physics/` is exported from the matching simulator configuration and robot asset. `controller.json` contains the source joint mapping, nominal gains, defaults, action scales and reset state. No physical-robot gain overrides or network services are used.

`references/manifest.json` lists all 38 source clips: 8 H2, 13 G1, 7 exploratory G1, 8 SMPL and 2 generated speech gestures. All are self-contained typed JSON targets at their original 50 Hz, not simulated states. Only H2 entries have separate FK reference previews. G1 and SMPL retain their native representation and use selectors 3 and 2 respectively.

`planner-bank/manifest.json` records the identical source neural-planner hash and the existing 29 offline-generated direction/style/speed entries. The browser executes those targets with the new released H2 policy. Old unselected reference files and compact-encoder fixtures are historical source records and are excluded from the static runtime dependency graph.

See `../README.md` for controls, regeneration and the current 114-case parity and complete-motion validation reports.
