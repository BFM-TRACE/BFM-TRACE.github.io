# Third-party software and asset provenance

This page records the licenses included with the browser runtimes and the recorded origins of the research assets. It does not assign one license to the entire website bundle or grant an additional MIT license for robot models, motion data, checkpoint weights, paper figures, or videos.

## Browser runtimes

| Component | Bundled version | Included license and notices |
| --- | --- | --- |
| Three.js, including OrbitControls | 0.180.0 | MIT; [THREE-LICENSE](demo/vendor/THREE-LICENSE) retains the Three.js authors' copyright notice. |
| ONNX Runtime Web | 1.30.0 | MIT; [LICENSE](demo/vendor/ort/LICENSE) and [ThirdPartyNotices.txt](demo/vendor/ort/ThirdPartyNotices.txt). The [runtime manifest](demo/vendor/ort/provenance.json) records the package archive, exact files and hashes. |
| MuJoCo WebAssembly bindings | 3.10.0 | Apache License 2.0; [LICENSE](demo/vendor/mujoco/LICENSE). The [runtime README](demo/vendor/mujoco/README.md) records the official package and upstream source links. |

These runtime licenses apply to their respective software. They do not establish the licensing of policy weights, meshes, reference motions, or research media used with that software.

## Robot, model and motion assets

The following paths identify the source SONIC-H2 checkout or source data collection. They are provenance, not runtime filesystem dependencies. The browser uses the files shipped in `demo/local-assets/` and `demo/assets/`. Detailed audits and export scripts marked **source checkout** below are present under `homepage/` in the source repository, but are not included in the static publication bundle. Their paths are intentionally plain text; linked runtime records remain available in the bundle.

| Bundled material | Recorded source | Provenance records |
| --- | --- | --- |
| H2 closed-loop XML model, STL meshes and visual geometry | `SONIC_unitree_h2/assets/robots/h2/H2_loop.xml`; matching simulator configuration | [Physics configuration](demo/local-assets/physics/physics-config.json), [physics manifest](demo/local-assets/physics/manifest.json), [visual model](demo/assets/h2-model.json) |
| Released universal encoder and H2 decoder | `SONIC_unitree_h2/models/sonic_h2_released` | [Model signatures and hashes](demo/local-assets/models/manifest.json), [source deployment manifest](demo/local-assets/models/source-deployment.json), [source license](demo/local-assets/models/SOURCE-LICENSE), [model attribution](demo/local-assets/models/SOURCE-NOTICE) |
| All 38 H2, G1, SMPL and generated speech-gesture reference clips | The five `SONIC_unitree_h2/motions/example_*_replay` collections | [Source inventory](demo/local-assets/source-inventory.json), [tracking manifest](demo/local-assets/references/manifest.json), [H2 FK preview manifest](demo/assets/motions.json) |
| Offline-generated planner reference bank | `SONIC_unitree_h2/models/motion_planner_g1/planner_sonic.onnx`, byte-identical to the original bank generator | [Bank manifest](demo/local-assets/planner-bank/manifest.json) |

The exact source revision and motion file hashes are recorded in the inventory. The source license and model attribution are retained with the released weights. Robot assets and reference data retain their respective source terms; the Three.js and ONNX Runtime MIT licenses do not relicense those materials. Earlier checkpoint and candidate audits are historical source records and are excluded from the static runtime bundle.

## Paper and demonstration media

The navigation's [arXiv logomark](assets/arxiv-logomark.svg) is the unmodified small primary SVG logomark from arXiv's [official brand assets](https://info.arxiv.org/brand/brand-guidelines.html) ([download](https://cornell.box.com/v/arxiv-logomark-small-svg)), displayed in its original red and gray colors with a transparent background to identify the link to this project's arXiv paper. arXiv retains its trademark rights.

The included manuscript, figures and demonstration clips come from this project's compiled paper and v33 presentation. Their source files, extraction steps and existing privacy treatment are recorded in source checkout: `content/research.json`, [media.json](content/media.json) and source checkout: `content/asset-validation.json`. The browser runtime licenses above do not license these research materials.

The homepage layout takes visual inspiration from the [ViBe project page](https://lok-i.github.io/vibe-control/). The homepage implementation is original; this attribution does not claim ownership of the referenced site's content.
