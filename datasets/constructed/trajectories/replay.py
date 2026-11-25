"""Read the compact JSONL shards and replay an episode in reference LIDAR_TOP coordinates."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path


AXES = ("x", "y", "z", "length", "width", "height", "yaw")


def apply(box, action):
    out = list(box)
    if action == ["STOP"]:
        return out
    axis, increment = action
    if not math.isfinite(increment) or increment == 0:
        raise ValueError(f"Invalid increment for {axis}")
    i = AXES.index(axis)
    out[i] += increment
    if i == 6:
        out[i] = (out[i] + math.pi) % (2 * math.pi) - math.pi
    if min(out[3:6]) <= 0:
        raise ValueError("Non-positive box dimensions")
    return out


def records(root, run):
    manifest = json.loads((root / "manifest.json").read_text())
    for shard in manifest["runs"][run]["shards"]:
        with gzip.open(root / shard["path"], "rb") as stream:
            for line in stream:
                yield json.loads(line)


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--run", choices=("full", "xyyaw"), default="full")
    parser.add_argument("--episode", help="ID to display; if omitted, show the first episode")
    parser.add_argument("--check", action="store_true", help="verify all compressed shard hashes in both runs")
    a = parser.parse_args()
    if a.check:
        runs = json.loads((a.root / "manifest.json").read_text())["runs"]
        for run in runs.values():
            for shard in run["shards"]:
                if file_hash(a.root / shard["path"]) != shard["sha256"]:
                    raise ValueError(f"SHA-256 mismatch: {shard['path']}")
        print("All shard hashes match.")
        return
    for ep in records(a.root, a.run):
        if a.episode and ep["episode_id"] != a.episode:
            continue
        box = ep["initial_box"]
        print(json.dumps({"episode_id": ep["episode_id"], "initial_box": box,
                          "status": ep["status"], "reason": ep["reason"]}))
        for step, action in enumerate(ep["actions"]):
            box = apply(box, action)
            print(json.dumps({"step": step, "action": action, "after_box": box}))
        return
    raise SystemExit("Episode ID was not found.")


if __name__ == "__main__":
    main()
