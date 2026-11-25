"""Build a self-contained, offline HTML review of 20 local teacher observations."""
import argparse
import base64
import json
from math import degrees
from pathlib import Path

from show20_geometry import overlays


CATEGORIES = dict(traffic_cone='交通锥', car='汽车', bus='公交车', truck='卡车',
                  construction_vehicle='工程车', pedestrian='行人', bicycle='自行车',
                  motorcycle='摩托车', trailer='挂车', barrier='隔离栏')

def instruction(a):
    if a['type'] == 'stop':
        return 'STOP · 保持当前框'
    v, axis = a['value'], a['axis']
    if axis == 'yaw':
        return f"俯视{'逆时针' if v>0 else '顺时针'}旋转 {abs(degrees(v)):.0f}°"
    cm = f'{abs(v)*100:g} cm'
    if a['type'] == 'translate':
        return f"沿{' +' if v>0 else ' −'}{axis} 平移 {cm}"
    name = {'length':'框长','width':'框宽','height':'框高'}[axis]
    return f"{name}{'增加' if v>0 else '减小'} {cm}"



def index(source, filename):
    with (source/filename).open(encoding='utf-8') as stream:
        return {row['id']:row for row in map(json.loads,stream)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, required=True,
                        help='Directory containing the JSONL manifests and locally generated views/')
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    source=args.source_dir.resolve()
    scripts=Path(__file__).resolve().parent
    tasks, observations, labels=(index(source,f'{name}.jsonl') for name in ('tasks','observations','labels'))
    ids=json.loads((scripts/'examples20.json').read_text())
    if len(ids)!=20 or len(set(ids))!=20:
        raise ValueError('Expected 20 unique observation IDs')
    review=source/'visual_spotcheck.json'
    notes={}
    if review.exists():
        checks=json.loads(review.read_text())
        notes={r['id']:r['visible_review'] for r in checks['records'] if r['id'] in checks['priority_ids']}
    gallery=[]
    for key in ids:
        t,o,r=tasks[key],observations[key],labels[key]
        if t['action']!=r['action'] or t['box']!=r['box'] or t['state_id']!=o['state_id']:
            raise ValueError(f'Inconsistent state/action for {key}')
        if len(o['images'])!=7 or r['images']!=o['images']:
            raise ValueError(f'Inconsistent seven-view input for {key}')
        images=[]
        for rel in o['images']:
            image=(source/rel).resolve()
            if image.parent!=(source/'views'/key).resolve() or image.suffix.lower()!='.jpg':
                raise ValueError(f'Unexpected image path: {rel}')
            images.append('data:image/jpeg;base64,'+base64.b64encode(image.read_bytes()).decode('ascii'))
        a=r['assessment']
        gallery.append(dict(id=key,scene=t['scene_name'],category=CATEGORIES[t['category']],
                            actionId=t['action_id'],stateId=t['state_id'],cmd=instruction(t['action']),
                            stopped=t['action']['type']=='stop',geo=overlays(t['box'],t['action']),
                            lidarFact=a['lidar_evidence'],cameraFacts=a['camera_evidence'],reason=a['rationale'],
                            support=a['support'],projected=o['projected'],review=notes.get(key,''),images=images))
    template=(scripts/'gallery20-template.html').read_text(encoding='utf-8')
    dataset=json.dumps(gallery,ensure_ascii=False,separators=(',',':')).replace('<',r'\u003c')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(template.replace('__GALLERY_DATA__',dataset),encoding='utf-8')
    print(f'Wrote {len(gallery)} cases to {args.output}')


if __name__=='__main__':
    main()
