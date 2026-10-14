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
| H2 closed-loop XML model, STL meshes and derived visual mesh bundle | `gear_sonic/data/assets/robot_description/unitree_ros/h2_description/H2_loop.xml` and its meshes | [Physics configuration](demo/local-assets/physics/physics-config.json), [physics file manifest](demo/local-assets/physics/manifest.json), [visual model](demo/assets/h2-model.json) |
| H2 encoder, H2 decoder and compact G1 encoder | `sonic_h2_native_release/experiments/gpu3_20260907_stage2_step009450/`; the compact G1 encoder is extracted from the full encoder's selector-3 branch | source checkout: `demo/local-assets/models/manifest.json`, source checkout: `demo/local-assets/models/encoder-g1-validation.json` |
| Native H2 tracking references and derived reference previews | `gear_sonic_deploy/reference/example_h2/deploy`; dynamic additions from the `bones_seed/h2_soma_full_0719_balanced/csv` collection | [Tracking manifest](demo/local-assets/references/manifest.json), source checkout: `demo/dynamic-motion-sources.json`, [preview manifest](demo/assets/motions.json) |
| Original G1 exploration references | `gear_sonic_deploy/reference/example_g1_explore` | source checkout: `qa/g1-explore-source-audit.json`, source hashes in each installed reference |
| Original G1 kicks and single-foot hopping reference | `gear_sonic_deploy/reference/example` | source checkout: `qa/g1-standard-source-audit.json`, source checkout: `qa/g1-standard-selection.json` |
| Offline-generated planner reference bank | Source planner `gear_sonic_deploy/planner/target_vel/V2/planner_sonic.onnx`; generated and screened with the recorded settings | [Bank manifest](demo/local-assets/planner-bank/manifest.json), source checkout: `demo/export_planner_bank.py` |

The repository records these assets' origins, transformations and hashes. A separate, verified redistribution license covering each of these robot, data and checkpoint assets is not included in this bundle. This notice does not replace their source terms, and the project does not relicense them under the MIT licenses bundled for Three.js or ONNX Runtime. Source names and hashes alone do not establish a license grant. No upstream raw-motion ownership or license is inferred when a legacy motion bundle does not declare it.

## Paper and demonstration media

The navigation's [arXiv logomark](assets/arxiv-logomark.jpg) is the unmodified small primary logomark from arXiv's [official brand assets](https://info.arxiv.org/brand/brand-guidelines.html), used to identify the link to this project's arXiv paper. arXiv retains its trademark rights.

The included manuscript, figures and demonstration clips come from this project's compiled paper and v33 presentation. Their source files, extraction steps and existing privacy treatment are recorded in source checkout: `content/research.json`, [media.json](content/media.json) and source checkout: `content/asset-validation.json`. The browser runtime licenses above do not license these research materials.

The homepage layout takes visual inspiration from the [ViBe project page](https://lok-i.github.io/vibe-control/). The homepage implementation is original; this attribution does not claim ownership of the referenced site's content.
