# 从本地 nuScenes 数据重建七图

使用 Python 3.10 或更高版本。准备通过 nuScenes 官方渠道取得的 `v1.0-trainval` 数据，解压目录应同时包含 `v1.0-trainval/` 元数据和 `samples/` 下的 LiDAR 与六路相机文件。以下命令假设当前目录包含本工具及 `requirements.txt`，将 `TEACHER` 换成含 `tasks.jsonl`、`observations.jsonl`、`labels.jsonl` 和复核 Notebook 的目录。

```bash
python -m pip install -r requirements.txt
NUSCENES=/path/to/nuscenes
TEACHER=/path/to/teacher1000
python render_views.py --dataroot "$NUSCENES" --tasks "$TEACHER/tasks.jsonl" --out "$TEACHER"
python build_gallery20.py --source-dir "$TEACHER" --output "$TEACHER/gallery20.html"
```

直接在浏览器打开本地 `gallery20.html` 即可查看选定的 20 个动作，并切换六路相机及动作前后框。HTML 内嵌了在本地重建的传感器图像。全部 1,000 个动作可在数据目录下打开复核 Notebook 查看。

渲染器使用每一步执行前的七参数框，按 `sample_token` 和 nuScenes 标定表重建投影。每台相机分别使用自身关键帧的自车位姿。输出写入 `views/<样本 ID>/`，已存在的原版观测清单会按逐步字段核对。匹配的已完成渲染可断点复用。各机器的字体、图像解码器及库版本可能影响 JPEG 编码字节。
