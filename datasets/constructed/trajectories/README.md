# nuScenes val 修框轨迹

两个目录使用相同的 nuScenes v1.0-trainval 官方 val 场景、相同的 CenterPoint 起始框和 GT 匹配。原始评估范围为 150 个场景和 6,019 个关键帧，其中 5,939 帧有匹配目标。每个目录均覆盖全部 113,005 个匹配框。这里以紧凑格式保存**全部成功及失败轨迹**。每一步的框可由初始框和动作序列重建。

| 版本 | 可编辑量 | 成功 | 失败 | 总动作（含 STOP） |
|---|---|---:|---:|---:|
| `full/` | x、y、z、长、宽、高、yaw | 111,917 | 1,088 | 935,147 |
| `xyyaw/` | x、y、yaw | 8,786 | 104,219 | 698,686 |

## 读取

从本目录使用 Python 3 标准库。预览第一条或指定轨迹时输出执行前框、动作和执行后框。两版中相同的 `episode_id` 对应相同的起始框和 GT 标注。

```bash
python3 replay.py --check
python3 replay.py --run full
python3 replay.py --run xyyaw --episode 000681a060c04755a1537cf83b53ba57_pred_000
```

每个 `episodes-*.jsonl.gz` 可独立解压和流式读取。分片包含多种场景，由 `status` 区分成功与失败。查找时请遍历所选版本的所有分片。

## 单条记录

| 字段 | 含义 |
|---|---|
| `episode_id`、`sample_token`、`scene_token` | 对象轨迹与 nuScenes 帧/场景的标识 |
| `reference_sample_data_token` | 本条记录所用的 LIDAR_TOP 帧 |
| `target_id`、`prediction_index`、`detection_score`、`category` | 检测目标、输入序号、置信度与类别 |
| `gt_annotation_token` | 匹配的官方 3D 框在 nuScenes `sample_annotation` 表中的 token |
| `initial_box` | 参考 LiDAR 坐标系的预测框，顺序为 `[x,y,z,length,width,height,yaw]`，中心及尺寸单位为米，yaw 单位为弧度 |
| `actions` | 顺序执行的动作列表。每次编辑为 `[axis,signed_increment]`，成功轨迹最后一步为 `["STOP"]` |
| `status`、`reason` | 轨迹终止情况。失败轨迹也保留所有实际执行过的动作 |

平移发生在参考 LiDAR 坐标系。长、宽、高分别沿框的局部轴修改总尺寸；单次尺寸编辑保持几何中心固定。yaw 绕 +z 按右手定则旋转，结果回绕至 `[-π,π)`。动作列表以 0 起算，第 i 条对应源轨迹的 `aNNN`（例如第 0 条为 `a000`）。非 STOP 动作把 `sNNN` 更新为下一状态，STOP 的前后状态相同。失败轨迹可以包含零步。

原轨迹停止条件为 BEV IoU 与 3D IoU 均不低于 0.85，平面中心误差不超过 0.15 m、z 误差不超过 0.10 m、最大尺寸误差不超过 0.15 m，以及适用类别的 yaw 误差不超过 5°。交通锥最小可编辑平移步长为 0.01 m，忽略 yaw；隔离栏 yaw 以 π 为周期。各轴档位、类别例外与生成配置见 `manifest.json`。其中 GT token、动作和终止理由属于轨迹监督信息，模型看图预测时应从观测输入中移除。

## 核验与导出

导出时逐步重放所有动作，与原源文件每步状态逐量比较并核验冻结轴、动作符号、档位、步序与终止位置；两次运行的初始框和匹配标识逐一对齐。源文件、分片和拼接后未压缩 JSONL 的 SHA-256 均记录在 `manifest.json`。从具有原计划器 `data1.jsonl`、`failed_episodes.jsonl`、配置和摘要的两个目录重建时使用

```bash
python3 export.py --full PATH_TO_FULL_RUN --xyyaw PATH_TO_XYYAW_RUN --output NEW_DIRECTORY
```

动作与起始框保留源文件中的数值精度，回放容差为 1e-10。当前紧凑形式保存 GT 的查找标识，完整官方 GT 框可依据 token 从相应版本 nuScenes 读取；逐步奖励和 IoU 属于原版完整审计文件。数据源及衍生内容适用 [nuScenes 数据使用条款](https://www.nuscenes.org/terms-of-use) 和 [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)。
