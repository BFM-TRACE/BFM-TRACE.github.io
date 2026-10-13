# TRACE project homepage

Project homepage for **TRACE: Efficient Transfer Learning for Neural Whole-Body Control Across Humanoid Embodiments**. Website code, media, policy weights, and browser simulation dependencies are contained in this directory. The layout takes visual inspiration from [ViBe](https://lok-i.github.io/vibe-control/); the implementation is original.

## 拷贝到另一台电脑展示

**完整复制 `homepage` 文件夹即可。** 目录包含视频、论文 PDF、H2 模型、策略权重、动作目标和浏览器运行库。不依赖原工程、移动硬盘、CDN 或 SSH；不要只复制 `index.html`，也不要漏掉 `demo/local-assets` 或 `demo/vendor`。

目标电脑安装 **Node.js 20 或更新版本**，然后任选一种方式启动：

- Windows：双击 `start-windows.cmd`。
- macOS：双击 `start-macos.command`。如果复制后丢失执行权限，在终端进入此目录，执行 `sh start-macos.command`。
- Windows / macOS / Linux 通用：在此目录打开终端，执行 `node server.mjs`。

打开 **http://127.0.0.1:4173**，展示期间保持启动窗口运行；按 `Ctrl+C` 停止。不需要 `npm install`，也不需要 Python、FFmpeg、CUDA 或单独安装 MuJoCo。不要直接双击 HTML：浏览器需要通过 HTTP 加载模块、WASM 和模型文件。如果端口 4173 已被占用，先关闭之前的启动窗口。

使用支持 WebAssembly SIMD、模块 Worker 和 WebGL 2 的现代桌面浏览器。本机已用 Chromium 验证；不同系统、浏览器和低性能设备的速度需要实测。首次打开交互区域时，浏览器需要加载三个模型，共 **74,626,647 bytes（约 74.6 MB）**，以及物理和推理运行库，请等待 **Browser policy + physics** 状态出现。文件从本机读取，复制完整后可断网展示。无需 WebGPU 或独立显卡。

### 本地交互演示如何运行

- **Motion tracking：** H2 原生参考使用 H2 encoder；m21 的 G1 参考使用同一模型的 G1 selector 3 编码分支，再交给 H2 decoder。在浏览器 Worker 中执行真实推理，每次读取当前物理状态，以 50 Hz 生成关节目标。原 H2 闭链模型在 MuJoCo WASM 中以 1 ms 步长计算接触和动力学，画面来自仿真结果。
- **Planner control：** 提供 `idle`、`slow_walk`、`walk`、`run` 四种模式。原 SONIC 神经规划器离线生成了各方向和速度档的参考。当前库包含 29 项目标，其中低速档采用经过闭环验证的完整步态与受限步频调整，避免低幅参考使 H2 原地不动。本地规划器组合这些原生 G1 目标，使用已验证等价于完整 encoder selector 3 的 compact G1 encoder，再由 H2 decoder 与物理反馈执行。网页没有加载约 739 MiB 的完整神经规划器，也没有实现原规划器的全部 27 种模式。
- **View reference only：** 用户明确选择后，单独查看参考动作。该模式不执行策略推理或物理反馈，界面会持续显示 reference-only 标签，规划控制禁用。

进入交互区域后，选择动作或切换到 **Planner control** 并选择模式。W/S 请求前进和后退，A/D 使用对应侧移动作目标，Q/E 独立控制朝向；支持 W+A 等斜向输入及移动与转向组合。松开单个按键会保留其他仍按住的输入，全部松开后停止请求。速度滑块和方向数字是目标，**Measured speed** 是实际仿真测量值，不保证实际速度精确等于目标。

已开放的**速度指令范围**如下；这些是规划指令，不是实测速度保证。高速松键后需要物理减速，不能瞬间停止。

| 模式 | 纯前进 | 含侧移或后退的输入 |
| --- | --- | --- |
| Slow walk | 0.1–0.8 m/s | 最大 0.8 m/s |
| Walk | 0.8–2.5 m/s | 最大 1.6 m/s |
| Run | 2.5–4.0 m/s | 最大 2.5 m/s |

**转向速度滑条（Turn rate command）** 独立设置 Q/E 或左右转向按钮的转速大小：Slow walk / Walk 为 **0.1–1.0 rad/s**，Run 为 **0.1–0.3 rad/s**，步长 0.1。按住转向时拖动会即时改变指令；松开停止转向。切换到 Run 会将超出上限的选择收至 0.3，较小选择保持不变；Idle 禁用。高速跑步的更大转速候选在闭环检查中出现摔倒，因此没有开放。

Motion tracking 新增 **Standing long jump、High jump、Right high kick、Left high kick**。四条均为原生 H2 数据，完整动作后继续保持末帧至少 2 秒，检查实际接触、腾空/踢腿幅度、落地和稳定站立后才加入菜单；踢腿动作包含走近和走离。参考预览文件与策略目标都随目录打包。见 [动作验证记录](qa/dynamic-motions.json)，这是当前模型与初始状态下的仿真检查。

m21 的 `reference/example_g1_explore` 已按说明文件编号接入 **1 单膝跪起、2 双膝跪起、4 原地轻跳、5 立定跳远**。菜单保留 `M21 #` 编号。它们保留原始 G1 29 维参考及完整 50 Hz 时序，直接经 G1 编码分支驱动 H2，不进行关节伪投影。播放前固定用真实策略和物理保持起始参考 **0.5 秒**，再以原速执行完整动作；Pause/Resume 不会重复准备，Restart 会从准备阶段重新开始。

第 5 条跳远对启动历史敏感：直接启动及部分其他准备时长会失败，固定 0.5 秒流程通过当前完整动作和结束后 2 秒检查。这里复现的是原部署“播放前持续控制起始姿态”的语义；0.5 秒为浏览器固定配置，不是对任意初态或启动时间的成功保证。见 [源数据与模型审计](qa/g1-explore-source-audit.json)、[启动比较](qa/g1-explore-startup-ab.json) 和 [最终动作检查](qa/g1-explore.json)。这些 G1 条目提供实时物理跟踪，无 H2 参考动画预览；选中时 View reference only 会禁用。

m21 的原生 G1 演示另新增 **M21 · Right high kick、M21 · Left high kick** 和 **M21 · Right-foot hops**。当前服务器实际目录为 `reference/example`，限定搜索未找到 `reference/example_g1`。两条踢腿各 3.28 秒，以踢腿为主，没有旧版踢垃圾动作较长的走近、走离；继续使用原始 G1 目标、固定 0.5 秒真实控制准备及同一 H2 模型。动作选择与跟踪质量见 [候选比较](qa/g1-standard-selection.json)、[正式物理检查](qa/g1-standard.json) 和 [来源审计](qa/g1-standard-source-audit.json)。

选择右脚单脚跳是因为中段抬起的左脚保持离地，避开了镜像版间歇性的另一脚落地。前几个跳跃幅度较小：五个源节奏点均有实际无地面接触区间，其中后两次达到严格腾空门槛；不能把它描述成五次同等幅度的大跳。完整 9.98 秒原动作之后仍保持 2 秒站稳。见 [逐峰检查](qa/g1-standard-hop-detail.json)。

旧导出器强制 16-token 长窗口并使用预测末尾上下文，选入了会周期性弯腰的前跑片段。新版导出默认与部署一致的 9–11 token、固定随机种子、当前播放位置上下文及重规划/混合时序，再提取单个完整步态周期。多速度参考按支撑腿相位混合，而不是把一个前跑片段无限倍速。高档候选必须通过姿态、方向、实际位移和停车检查才会开放。Idle 和 Run 的后退/左右四项仍保留已通过闭环检查的历史参考，各文件保留原始生成来源；其余新速度档使用修正后的生成流程。

`local-planner.js` 管理相位对齐、方向和模式混合、起停过渡，并从同一条未来目标轨迹计算关节位置与速度。朝向与平移分别控制：先减去每个循环的平均 yaw，再将自然摆动限制在 ±0.08 rad，避免某些侧跑样本的大幅扭身覆盖用户朝向；根据物理朝向误差增加有界的目标修正。这些操作只修改控制目标，机器人姿态始终由策略、力矩和 MuJoCo 动力学产生。严格的朝向保持与模式切换效果以当前代码对应的闭环 QA 报告为准。

此前速度修复的验收通过了 16 项姿态/速度检查（含 60 秒连续前跑）、20 项速度与方向边界、15 项模式/方向矩阵、6 项组合输入和 23 项界面控制检查。旧前跑在物理仿真中的躯干俯仰峰值约 60°；修复后 2.5 m/s 指令的 60 秒前跑峰值约 8.4°。Walk 高速停车另做了五个松键相位检查，使用 0.4 秒目标过渡。详细条件与当前文件哈希见 [本次验收记录](qa/planner-speed-fix-summary.json)。

**Pause** 冻结当前物理状态，**Resume** 继续；动作结束后控制器保持末帧目标。**Reset / Restart local rollout** 以及 Motion tracking / Planner control 之间的切换会从初始状态开始新 rollout；Planner 内的步态样式切换通过目标混合过渡。页面进入后台时仿真暂停，离开页面会终止该页面的 Worker。每个页面独立运行，不会与其他访客共享控制状态。

初始化失败时模型保持静止并显示错误，不会自动换成动画。可以重试或明确选择参考预览。仿真中失去平衡时使用 Reset；慢设备会降低仿真相对于现实时间的速度，界面显示运行倍率。此前浏览器检查确认了本地执行和静态服务器上双实例的独立控制，未请求仿真 API；新版方向与模式闭环矩阵的 15 项检查全部通过，共 174 秒仿真，覆盖四种模式、前后左右移动、朝向保持、停止和模式往返切换。这些开发检查也不替代论文评测或保证其他设备的性能。

原服务器桥接工具保留在 [backend/README.md](backend/README.md) 中，供旧服务器方案维护使用。它不是本地演示的启动依赖；当前浏览器演示不调用该服务器或真实机器人。

## Development

The static server requires Node.js 20 or newer and has no npm dependencies.

```sh
cd homepage
npm run dev
```

Open **http://127.0.0.1:4173**; the standalone interactive view is at **http://127.0.0.1:4173/demo/**. `PORT` and `HOST` override the bind address. The server serves WASM with the correct MIME type, supports video seeking, and retains an optional legacy `/api/*` proxy. The browser simulation uses local Worker messages, not that proxy.

All runtime dependencies are served locally. ONNX Runtime Web uses single-threaded CPU WASM, so cross-origin isolation headers are not required. This compact bundle does not include a WebGPU provider. See [demo/README.md](demo/README.md) for the execution contract and [ORT's deployment guide](https://onnxruntime.ai/docs/tutorials/web/deploy.html) for hosting requirements.

## 静态发布与 GitHub 存储

在 `homepage` 目录执行：

```sh
npm run check:release
```

这会检查资源和控制逻辑，生成 `dist/`，再检查发布包的文件哈希、子路径访问、WASM MIME 类型和视频 Range 请求。没有 npm 依赖，不需要先执行 `npm install`。

**静态网站仅发布 `dist/` 的全部内容。** 它包含页面、视频、论文、模型、动作数据及浏览器运行库，可放到支持 HTTP 的静态托管服务；不需要 Node 后端、SSH 或仿真服务器。复制或部署时保留目录结构和 `.nojekyll`，使用支持 `.wasm` MIME 类型及视频 Range 请求的服务器。不能直接通过 `file://` 打开。`dist/build-manifest.json` 记录文件大小和 SHA-256，便于确认拷贝完整。

`dist/` 是可重建产物，Git 忽略该目录，仓库保留完整源目录和运行资产。直接拷贝整个 `homepage` 到其他电脑的启动方式仍是上面的 `node server.mjs`；只上传到 GitHub 仓库不会自动启用网站托管。

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
