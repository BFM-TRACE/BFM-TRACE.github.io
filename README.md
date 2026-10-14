# TRACE project homepage

Project homepage for **TRACE: Efficient Transfer Learning for Neural Whole-Body Control Across Humanoid Embodiments**. Website code, media, policy weights, and browser simulation dependencies are contained in this directory. The layout takes visual inspiration from [ViBe](https://lok-i.github.io/vibe-control/); the implementation is original.

## 拷贝到另一台电脑展示

**完整复制 `homepage` 文件夹即可。** 目录包含视频、论文 PDF、H2 模型、策略权重、动作目标和浏览器运行库。不依赖原工程、移动硬盘、CDN 或 SSH；不要只复制 `index.html`，也不要漏掉 `demo/local-assets` 或 `demo/vendor`。

目标电脑安装 **Node.js 20 或更新版本**，然后任选一种方式启动：

- Windows：双击 `start-windows.cmd`。
- macOS：双击 `start-macos.command`。如果复制后丢失执行权限，在终端进入此目录，执行 `sh start-macos.command`。
- Windows / macOS / Linux 通用：在此目录打开终端，执行 `node server.mjs`。

打开 **http://127.0.0.1:4173**，展示期间保持启动窗口运行；按 `Ctrl+C` 停止。不需要 `npm install`，也不需要 Python、FFmpeg、CUDA 或单独安装 MuJoCo。不要直接双击 HTML：浏览器需要通过 HTTP 加载模块、WASM 和模型文件。如果端口 4173 已被占用，先关闭之前的启动窗口。

使用支持 WebAssembly SIMD、模块 Worker 和 WebGL 2 的现代桌面浏览器。本机已用 Chromium 验证；不同系统、浏览器和低性能设备的速度需要实测。首次打开交互区域时，浏览器需要加载新版完整 encoder 和 decoder，共 **107,061,556 bytes（约 107.1 MB）**，以及物理和推理运行库，请等待 **Browser policy + physics** 状态出现。文件从本机读取，复制完整后可断网展示。无需 WebGPU 或独立显卡。

### 本地交互演示如何运行

- **Motion tracking：** 全部权重和 38 条动作来自 M21 的 `~/workspace/SONIC_unitree_h2` 发布版本。原生 H2、G1、SMPL 参考分别使用完整 encoder 的 selector 0、3、2，再交给同一 H2 decoder。在浏览器 Worker 中执行真实推理，每次读取当前物理状态，以 50 Hz 生成关节目标。原 H2 闭链模型在 MuJoCo WASM 中以 1 ms 步长计算接触和动力学，画面来自仿真结果。
- **Planner control：** 提供 `idle`、`slow_walk`、`walk`、`run` 四种模式。原 SONIC 神经规划器离线生成了各方向和速度档的参考。当前库包含 29 项目标，其中低速档采用经过闭环验证的完整步态与受限步频调整，避免低幅参考使 H2 原地不动。本地规划器组合这些原生 G1 目标，使用发布版完整 encoder 的 selector 3，再由 H2 decoder 与物理反馈执行。网页没有加载约 739 MiB 的完整神经规划器，也没有实现原规划器的全部 27 种模式。
- **View reference only：** 用户明确选择后，单独查看参考动作。该模式不执行策略推理或物理反馈，界面会持续显示 reference-only 标签，规划控制禁用。

进入交互区域后，选择动作或切换到 **Planner control** 并选择模式。W/S 请求前进和后退，A/D 使用对应侧移动作目标，Q/E 独立控制朝向；支持 W+A 等斜向输入及移动与转向组合。松开单个按键会保留其他仍按住的输入，全部松开后停止请求。速度滑块和方向数字是目标，**Measured speed** 是实际仿真测量值，不保证实际速度精确等于目标。

已开放的**速度指令范围**如下；这些是规划指令，不是实测速度保证。高速松键后需要物理减速，不能瞬间停止。

| 模式 | 纯前进 | 含侧移或后退的输入 |
| --- | --- | --- |
| Slow walk | 0.1–0.8 m/s | 最大 0.8 m/s |
| Walk | 0.8–2.5 m/s | 最大 1.6 m/s |
| Run | 2.5–4.0 m/s | 最大 2.5 m/s |

**转向速度滑条（Turn rate command）** 独立设置 Q/E 或左右转向按钮的转速大小：Slow walk / Walk 为 **0.1–1.0 rad/s**，Run 为 **0.1–0.3 rad/s**，步长 0.1。按住转向时拖动会即时改变指令；松开停止转向。切换到 Run 会将超出上限的选择收至 0.3，较小选择保持不变；Idle 禁用。高速跑步的更大转速候选在闭环检查中出现摔倒，因此没有开放。

当前菜单完整接入新仓库的 **38 条 reference**：8 条 H2、13 条基础 G1、7 条探索 G1、8 条 SMPL，以及 2 条生成的语音手势动作（网页播放动作，不播放语音）。它们保留源库原始 50 Hz 时序；H2 提供独立 FK 参考预览，G1/SMPL 通过真实策略和物理执行。每条动作开始前保持首帧 0.5 秒（右脚连续跳为 1.0 秒），结束后保持末帧；这是网页的播放协议，源命令行 replay 结束后释放控制并回到站立。

新版模型为 `SONIC_unitree_h2/models/sonic_h2_released`，三个源 ONNX 文件哈希均与发布清单一致；浏览器实际加载完整 encoder 和 decoder。新仓库内的神经 planner 与旧参考库的生成模型字节一致，因此保留其 29 项离线参考及交互调度；策略权重和物理配置已换成新发布版本。来源、逐文件哈希和导出过程见 `demo/local-assets/source-inventory.json` 与 `demo/export_m21_release.py`。

当前验证入口为 `node scripts/check-m21-parity.mjs`（114 个源 Python/CPU ONNX 与网页 WASM 的观测、tokens、decoder 对照）、`node scripts/check-m21-release.mjs`（全部 38 条动作及行走回归），结果写入 `qa/generated/m21-parity.json`、`qa/generated/m21-release.json`。较早的 `qa/g1-*`、`qa/dynamic-*` 和 9450 checkpoint 审计是历史记录，不能作为新模型的验证证据。

### 验证记录与归档

`qa/` 保留验收指标、协议、来源、截图和历史失败记录。九份较大的完整逐帧报告已移至原工作区的 `.work/archive/homepage-qa-20261011/`；仓库内同名文件保留去掉逐帧 `samples` 的摘要。原始报告与摘要的大小、哈希及归档位置见 [归档索引](qa/archive-index.json)。原始报告哈希与摘要哈希明确区分，归档不是运行或复现检查的依赖。

常用 `npm run check:*` 检查将新报告写入被 Git 忽略的 `qa/generated/`，保留已审定摘要不被覆盖。G1 标准动作的源测量文件仍随仓库保留，`check:g1` 和 `check:g1-standard` 可直接使用本目录资产复现；无需原服务器。生成的完整报告需要人工审阅后再更新仓库中的验收摘要。

第三方运行库、机器人资产和策略的来源及已有许可文件见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## Content and provenance

- Full-screen real-robot video, paper and project links. The hero uses the complete 20.633-second door clip, including the doorway traversal, at original speed; see [media validation](qa/hero-video.json).
- Browser-local H2 motion tracking and a lightweight gait planner.
- Complete 2:50 narrated video with English captions.
- Current paper's method figure, four adaptation stages, PHUMA evaluation, and downstream VLA task results.
- 17 original-speed hardware clips: 8 tracking, 5 teleoperation, and 4 VLA demonstrations, extracted from the v33 deck with its existing privacy treatment.
- Responsive desktop/mobile layouts, keyboard navigation, reduced-motion support, and deferred media loading. The layout supports mobile; local simulation performance on mobile is not guaranteed.

The site serves `assets/trace-paper.pdf`, synchronized from `build/en/main_en.pdf`: the current nine-page manuscript with author affiliations, the corresponding-author legend, and the project-page link, reporting **approximately 8 hours** of adaptation. Download links include a PDF hash version to refresh cached copies. The older root `TRACE.pdf` reports 6 hours and is not used. No venue acceptance is inferred. PHUMA success rates are labelled as simulation results with height-scaled tolerances. Demonstration clips and VLA benchmark trials are described separately.

`content/research.json` records scientific values and source provenance; `content/media.json` indexes the videos. `content/asset-validation.json` records media hashes and extraction checks. The website's headline values and result tables are explicit HTML for search indexing, so update them together with `research.json` when revising results. Browser policy and physics provenance is stored in `demo/local-assets/` and `demo/vendor/`.

The top-right **Arxiv** entry links to [arXiv:2610.17999](https://arxiv.org/abs/2610.17999). The hero's **Read the paper** button keeps its bundled PDF link. The bottom Citation section uses the [official arXiv BibTeX export](https://arxiv.org/bibtex/2610.17999), with a copy button supporting both HTTPS hosting and the HTTP LAN deployment.

## Rebuild media and verify

The delivered website already includes all media. Regenerating media requires Python with Pillow, FFmpeg/FFprobe, Poppler's `pdftoppm`, and the original v33 PPT and compiled paper in their project locations. These are development dependencies only.

From the original project root:

```sh
python3 homepage/scripts/prepare_assets.py
cd homepage
npm run check
npm run check:physics
npm run check:simulation
```

The site check verifies local resources, navigation anchors, gallery counts, HTTP video seeking, private-file exclusions, and legacy command-body forwarding against an isolated fake backend. It does not replace browser physics validation. Execution and numerical checks are recorded under `qa/`, `demo/`, and `backend/` with their own scope.

Current planner evidence is separated by scope:

- [Original G1 kicks, single-leg hop and full hero update](qa/g1-standard-update.json): installed sources, physical tracking checks, browser observations and the complete door-video provenance.
- [m21 update summary](qa/g1-explore-update.json): current implementation hashes, four G1 motions, four existing H2 dynamic motions, 15 planner cases and 44 controls checks.
- [m21 G1 exploration tracking](qa/g1-explore.json): the four requested original references, typed encoder routing, fixed live startup hold and post-motion standing. [Browser evidence](qa/g1-explore-browser.json) covers real interaction and renderer output.

- [Earlier turn slider and dynamic-motion update](qa/turning-and-dynamic-summary.json): source hashes at that revision, 33 UI checks, 24 physical turning cases, four complete dynamic actions and [browser observations](qa/turning-and-dynamic-browser.json).

- [Model-chain audit](qa/model-chain-audit.md): exact model hashes, selector and physics-parameter parity with the running step9450 simulation; the separate hardware launcher is configured for step70000.
- [Posture and speed quality](qa/planner-quality.json): long forward running, supported speed response, body/waist pitch, whole-cycle excursion and braking. Low-speed motion must actually make progress; an upright stationary robot fails.
- [Directional boundaries](qa/planner-boundaries.json): slow endpoints, walking/running limits, and running diagonals, including heading and physical stopping.
- [Compact-bank parity](qa/planner-compaction-parity.json): 10,000 sampled future references have exactly equal q/qdot/quaternions before and after trimming unused frames. The runtime bank is approximately 3.9 MB.


- [G1 encoder extraction and equivalence](demo/local-assets/models/encoder-g1-validation.json): 473 random and real-reference inputs match full encoder selector 3 bit for bit; [local WASM verification](demo/local-inference-g1-wasm-qa.json) also matches 14 stored fixtures.
- [Planner controls](qa/planner-controls.json): 44 DOM event-harness cases cover combined keys, independent turning, releases, style selection, typed H2/G1 routing, preparation timing and pause/restart behavior. They do not test physics or a real browser.
- [Planner bank closed-loop report](qa/planner-bank.json): actual simulated displacement, heading, stopping and style transitions. All 15 bounded cases pass (174 simulated seconds), including signed displacement with the requested axis dominant, heading drift below 0.3 rad, and final-second stop RMS below 0.25 m/s. See the exact limits and source hashes; this is not a universal guarantee. [The report before the facing fix](qa/planner-bank-before-facing-fix.json) is retained explicitly as historical failed evidence.

- [Combined planner inputs](qa/planner-combinations.json): all 6 bounded diagonal/translation-plus-turn physics cases pass, including stable stopping.
- [Turn-rate checks](qa/planner-turn-rates.json): signed turning and final-second settling after a six-second stop observation, using real policy inference and contact dynamics. The [four-second report](qa/planner-turn-rates-stop4s.json) is retained: Walk at 2.5 m/s with −0.1 rad/s had a brief heading oscillation and settled during seconds 5–6. Stop thresholds and controller parameters were not relaxed to hide it.
- [Dynamic motion checks](qa/dynamic-motions.json): all four additions execute their full native H2 reference plus a two-second final hold; actual flight, landing, kicking and final stability are measured.
- [Browser interaction check](qa/planner-browser.json): actual keyboard controls, idle/run selection and tracking/planner switching, with a saved screenshot and no captured console warnings or errors.

From `homepage`, run `node demo/check_g1_encoder.mjs`, `node scripts/check-planner-controls.mjs`, or `node scripts/check-planner-bank.mjs` to regenerate the respective checks. `npm run check:quality`, `npm run check:boundaries`, `npm run check:planner` and `npm run check:combinations` run the closed-loop movement checks; `npm run check:controls` checks UI input handling. `npm run check:turning` checks supported turn rates; `npm run check:dynamic` checks the four native H2 dynamic additions; `npm run check:g1` checks the four m21 G1 exploration references; `npm run check:g1-standard` checks the original G1 kicks and the selected right-foot hopping sequence. These execute real model inference and physics where stated and take longer than a static resource check.

## Files

| Path | Purpose |
| --- | --- |
| `index.html`, `styles.css`, `app.js` | Homepage and media interactions |
| `server.mjs`, `start-windows.cmd`, `start-macos.command` | Portable local startup and static serving |
| `demo/` | Three.js viewer, Worker controller, local gait planner, inference and physics modules |
| `demo/local-assets/` | H2/G1 encoders, H2 decoder, multi-speed planner bank, controller configuration and physics model |
| `demo/vendor/` | Local Three.js, ONNX Runtime Web and MuJoCo runtimes and licenses |
| `backend/` | Legacy server simulation bridge and connection instructions |
| `assets/` | Project videos, posters, figures, paper and captions |
| `content/` | Source-backed research and media manifests |
| `scripts/` | Media preparation, static build and site verification |
| `dist/` | Generated static deployment bundle; ignored by Git |
| `qa/` | Reviewed validation summaries, source measurements and screenshots |
| `qa/generated/` | New validation output; ignored by Git |

The website does not modify the existing paper, presentation, or demonstration deliverables. Public deployment is separate from local startup.
