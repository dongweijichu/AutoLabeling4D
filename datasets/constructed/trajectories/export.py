"""Compact the two original planner runs while verifying every source state."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

from replay import AXES, apply, file_hash


def line(record):
    return (json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n").encode()


class Shards:
    def __init__(self, root, name):
        self.root = root
        self.name = name
        self.raw = None
        self.stream = None
        self.output = []
        self.rows = 0
        self.uncompressed = 0

    def finish_shard(self):
        if self.stream is not None:
            self.stream.close()
            self.raw.close()
            self.output.append(dict(path=str(self.path.relative_to(self.root)), sha256=file_hash(self.path),
                                    bytes=self.path.stat().st_size, episodes=self.rows))
            assert self.path.stat().st_size < 24 * 1024**2
            self.stream = None

    def add(self, data):
        if self.stream is None or self.uncompressed + len(data) > 20 * 1024**2:
            self.finish_shard()
            self.path = self.root / self.name / f"episodes-{len(self.output):03d}.jsonl.gz"
            self.path.parent.mkdir(exist_ok=True)
            self.raw = self.path.open("xb")
            self.stream = gzip.GzipFile(filename="", mode="wb", compresslevel=6, fileobj=self.raw, mtime=0)
            self.rows = self.uncompressed = 0
        self.stream.write(data)
        self.uncompressed += len(data)
        self.rows += 1


def compact(ep, expected_status, conf, axes, totals):
    def require(condition):
        if not condition:
            raise ValueError(f"Invalid source state/action in {ep['episode_id']}")
    states = ep["states"]
    transitions = ep["transitions"]
    end = ep["termination"]
    require(ep["source_split"] == "val" and ep["coordinate_frame"] == "reference_lidar")
    require(end["status"] == expected_status and ep["source"]["detector"] == "centerpoint")
    success = expected_status == "success"
    require(len(transitions) == len(states)-1+int(success))
    require(ep["initial_state_id"] == states[0]["state_id"] and ep["terminal_state_id"] == states[-1]["state_id"])
    box = states[0]["box"]
    require(len(box)==7 and all(map(math.isfinite, box)))
    acts = []
    for i,t in enumerate(transitions):
        a = t["action"]
        require(t["step"] == i and t["action_id"] == f"a{i:03d}" and t["from_state_id"] == states[i]["state_id"] == f"s{i:03d}")
        stop = a["type"] == "stop"
        require(stop == (success and i == len(transitions)-1))
        if stop:
            require(a == {"type":"stop"} and t["to_state_id"] == t["from_state_id"])
            action = ["STOP"]
        else:
            axis, value = a["axis"], a["value"]
            require(axis in axes and axis in AXES)
            require(a["type"] == ("translate" if axis in ("x","y","z") else "rotate" if axis=="yaw" else "resize"))
            require(not (ep["category"] == "traffic_cone" and axis == "yaw"))
            label = t["action_label"]
            level = label["level"]
            category_levels = conf.get("category_overrides", {}).get(ep["category"], {}).get(axis, {})
            magnitude = category_levels.get(level, conf["action_values"][axis][level])
            require(label["axis"] == axis and label["direction"] == ("increase" if value > 0 else "decrease"))
            require(math.isclose(abs(value), magnitude, rel_tol=0, abs_tol=1e-12))
            action = [axis, value]
            box = apply(box, action)
            require(t["to_state_id"] == states[i+1]["state_id"] == f"s{i+1:03d}")
            require(all(abs(x-y) <= 1e-10 for x,y in zip(box,states[i+1]["box"])))
            if axes == ("x", "y", "yaw"):
                require(states[i+1]["box"][2:6] == states[0]["box"][2:6])
        totals[action[0]] += 1
        acts.append(action)
    keys = ("episode_id","sample_token","scene_token","reference_sample_data_token","target_id","category")
    return dict({k:ep[k] for k in keys}, prediction_index=ep["source"]["prediction_index"],
                detection_score=ep["source"]["score"], gt_annotation_token=ep["supervision"]["gt_annotation_token"],
                initial_box=states[0]["box"], actions=acts, status=expected_status, reason=end["reason"])


def export_run(folder, root, name):
    conf = json.loads((folder / "trajectory_config.json").read_text())
    summary = json.loads((folder / "summary.json").read_text())
    src_hash = {}
    totals, counts, reasons = Counter(), Counter(), Counter()
    pairs = {}
    frames, scenes = set(), set()
    axes = ("x","y","yaw") if name == "xyyaw" else AXES
    shards = Shards(root, name)
    exported_hash = hashlib.sha256()
    for filename, status in (("data1.jsonl","success"),("failed_episodes.jsonl","failure")):
        digest = hashlib.sha256()
        with (folder / filename).open("rb") as f:
            for raw in f:
                digest.update(raw)
                item = compact(json.loads(raw), status, conf, axes, totals)
                key = item["episode_id"]
                if key in pairs:
                    raise ValueError("Repeated episode: "+key)
                pairs[key] = hashlib.sha256(line([item[k] for k in ("sample_token","scene_token","reference_sample_data_token","target_id","category","prediction_index","detection_score","initial_box","gt_annotation_token")])).digest()
                frames.add(item["sample_token"])
                scenes.add(item["scene_token"])
                counts[status] += 1
                reasons[item["reason"]] += 1
                data = line(item)
                shards.add(data)
                exported_hash.update(data)
        src_hash[filename] = digest.hexdigest()
    shards.finish_shard()
    assert counts["success"] == summary["overall"]["success"] and counts["failure"] == summary["overall"]["failed"]
    assert sum(counts.values()) == summary["scope"]["matched_boxes"] == 113005
    assert len(frames) == summary["frame_level"]["frames_with_matched_targets"] == 5939
    assert len(scenes) == summary["scope"]["scenes"] == 150
    actual = hashlib.sha256()
    for shard in shards.output:
        with gzip.open(root / shard["path"], "rb") as f:
            for block in iter(lambda:f.read(1024*1024), b""):
                actual.update(block)
    assert actual.digest() == exported_hash.digest()
    metadata_keys = ("box_convention","stop","max_edit_steps","action_values","category_overrides",
                     "traffic_cone_yaw","barrier_yaw","residual_clipping")
    d = dict(axes=list(axes), counts=dict(counts), total_actions=sum(totals.values()), action_counts=dict(totals),
             states=sum(totals.values())-totals["STOP"]+len(pairs), termination_reasons=dict(reasons),
             scope=summary["scope"], frames_with_matches=len(frames), thresholds=conf["stop"], configuration={k:conf[k] for k in metadata_keys if k in conf},
             source_hashes=src_hash, matched_input_sha256=conf["input_sha256"], stream_sha256=actual.hexdigest(),
             replay_checked=True, shards=shards.output)
    print(name, dict(counts), "actions", sum(totals.values()), "shards", len(shards.output), flush=True)
    return d, pairs


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for n in ("full","xyyaw","output"):
        p.add_argument("--"+n, required=True, type=Path)
    a = p.parse_args()
    a.output.mkdir(exist_ok=True)
    for name in ("full","xyyaw","manifest.json"):
        if (a.output/name).exists():
            p.error("Use a new output directory")
    runs = {}
    runs["full"], baseline = export_run(a.full, a.output, "full")
    runs["xyyaw"], restricted = export_run(a.xyyaw, a.output, "xyyaw")
    if baseline != restricted:
        raise ValueError("Different source identities / starting boxes across runs")
    manifest=dict(format="compact-box-actions/v1", dataset="nuScenes v1.0-trainval", source_split="val", detector="CenterPoint",
                  identical_pairs_and_initial_boxes=True, source_replay_tolerance=1e-10, runs=runs)
    (a.output/"manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+"\n")


if __name__ == "__main__":
    main()
