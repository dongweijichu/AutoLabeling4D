# CenterPoint 修框轨迹与七图教师标注

这组数据记录了从 CenterPoint 初始检测框逐步调整至 nuScenes GT 匹配框的过程。可以从三个入口开始。

| 要做的事 | 数据入口 | 内容 |
|---|---|---|
| 看一次动作在图上改了哪里 | [20 个离线可视化示例](teacher_1000/gallery20.html) | 下载 HTML，用浏览器打开，可切换 LiDAR、六路相机和动作前后框 |
| 核对教师的文字依据 | [七图复核 Notebook](teacher_1000/七图动作人工复核.ipynb) · [1,000 条标注](teacher_1000/labels.jsonl) | 每条有执行前的七张图、轨迹动作、教师引用的可见证据和支持程度 |
| 逐步重放任意匹配框 | [轨迹说明与读取命令](trajectories/README.md) | 七参数版和仅 x、y、yaw 版各含 113,005 条轨迹，保留所有成功与失败记录 |

## 轨迹与抽样

检测与 GT 匹配使用 nuScenes v1.0-trainval 的官方 val 划分，覆盖 150 个场景、6,019 帧，其中 5,939 帧有匹配框。七参数版可调 x、y、z、长、宽、高、yaw，共有 111,917 条成功轨迹。仅调整 x、y、yaw 时，同一批起始框中有 8,786 条达到相同停止条件。各轴步长、成功条件、失败原因及逐步回放方法见轨迹目录。

1,000 个复核动作从七参数版的成功轨迹抽取，来自 142 个场景、1,000 个不同帧和不同轨迹。七个轴各抽了 130 步，另有 90 个 STOP。此分布由抽样配额决定。若把这些数据用于训练，请按场景另行划分训练和评估集，并报告使用过的官方 val 场景。

## 复核教师标注

在 `teacher_1000/` 下将三个 JSONL 按 `id` 关联。`tasks.jsonl` 提供当前框、轨迹标识与参考动作，`observations.jsonl` 列出执行前的七张图、相机投影和点云计数，`labels.jsonl` 存储教师给出的可见依据与判断。回查轨迹时用 `episode_id` 定位对象，再用从零开始的 `step` 定位动作。图像路径均相对于 `teacher_1000/`。

教师在看到参考动作后判断七图能支持它到什么程度。1,000 条中，21 条标为支持，198 条为部分支持，781 条为证据不足。这些等级来自模型。人工复核时，可先在上述 Notebook 把 `SHOW_TEACHER` 和 `SHOW_ACTION` 设为 `False`，自己判断后再对照。用 Linux 或 macOS 的 Python 3 Jupyter 内核，从 Notebook 所在目录运行即可。[已有抽查记录](teacher_1000/visual_spotcheck.json) · [从原始 nuScenes 重建七图](teacher_1000/tools/USAGE.md)

生成时使用的固定中文提示词随包提供。每个批次另附样本的执行前拼图、候选框和既定动作。`sampling.json` 与 `verification.json` 的 `source` 标识原始七参数版成功轨迹文件；公开版中同一条轨迹可通过 `episode_id` 在 `trajectories/full/` 找到。[教师提示词](teacher_1000/teacher_prompt_zh.txt)

## 早期试验

模拟点云教程有 20 和 200 个训练场景两个版本，共用同一组开发与测试样本。早期真实七图试验源于 nuScenes 官方 train 划分，包含 50 个训练候选和 33 个开发候选，保留 50 条教师原始回答，其中 11 条 STOP 通过当时的筛选。[数据、图像及结果说明](early/README.md)

## 完整性核验

在 Linux 上进入本目录，运行以下命令。第一条核对公开文件的 SHA-256；第二条核对两版轨迹分片。

```bash
sha256sum --quiet -c SHA256SUMS
python3 trajectories/replay.py --check
```

## 数据来源与许可

原始相机、LiDAR 和人工框来自 Motional 发布的 nuScenes。AutoLabeling4D 使用 CenterPoint 预测与官方标注配对，生成离散修框轨迹、动作标签、相机投影、点云渲染、裁剪图及教师文字标注。本目录中的 nuScenes 衍生材料依 CC BY-NC-SA 4.0 和随数据提供的 nuScenes 附加条款分享。使用时请保留署名与改动说明，遵守非商业和相同方式共享条件。[nuScenes 官方条款](https://www.nuscenes.org/terms-of-use) · [随源数据分发的条款全文](NUSCENES_TERMS.txt) · [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/) · [Caesar 等，nuScenes，CVPR 2020](https://openaccess.thecvf.com/content_CVPR_2020/html/Caesar_nuScenes_A_Multimodal_Dataset_for_Autonomous_Driving_CVPR_2020_paper.html)
