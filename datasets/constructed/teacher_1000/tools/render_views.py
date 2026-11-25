#!/usr/bin/env python3
"""Render a pre-action box in reference LiDAR coordinates and the six nuScenes keyframe cameras.

Usage: python render_views.py --dataroot NUSCENES_ROOT --tasks tasks.jsonl --out . --workers 4
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import ijson
import numpy as np
from PIL import Image, ImageDraw, ImageFont

CAMERAS = ('CAM_FRONT_LEFT','CAM_FRONT','CAM_FRONT_RIGHT','CAM_BACK_LEFT','CAM_BACK','CAM_BACK_RIGHT')
VIEWS = ('LIDAR', *CAMERAS)
# Bottom ring, top ring, then the vertical edges joining them.
SIGNS = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                  [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], dtype=float)
EDGES = [(i, (i//4)*4+(i+1)%4) for i in range(8)] + [(i,i+4) for i in range(4)]
ORANGE = '#ffa62d'
BG = '#14212b'
TEXT = '#f0f2f4'
HEAD = '#a6dced'


def jsonl(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def write_atomic(path, contents):
    with tempfile.NamedTemporaryFile(dir=path.parent,delete=False) as out:
        temp = Path(out.name)
        try:
            out.write(contents)
        except BaseException:
            temp.unlink(missing_ok=True)
            raise
    os.replace(temp, path)


def json_bytes(obj):
    return (json.dumps(obj,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode()


def lookup(path, tokens):
    with path.open('rb') as stream:
        records = {r['token']: r for r in ijson.items(stream,'item',use_float=True) if r['token'] in tokens}
    if records.keys() != tokens:
        raise ValueError(f'{path}: {len(tokens-records.keys())} missing tokens')
    return records


@lru_cache(maxsize=4)
def font(size):
    try:
        return ImageFont.truetype('DejaVuSans.ttf',size)
    except OSError:
        return ImageFont.load_default()


def rotation(pose):
    q = np.asarray(pose['rotation'],dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('Invalid nuScenes quaternion')
    w,x,y,z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def to_parent(points, pose):
    return points @ rotation(pose).T + np.array(pose['translation'])


def to_child(points, pose):
    return (points-np.array(pose['translation'])) @ rotation(pose)


def box_geometry(box):
    x,y,z,length,width,height,yaw = box
    if not np.isfinite(box).all() or min(length,width,height)<=0:
        raise ValueError('Invalid x y z length width height yaw')
    c,s = math.cos(yaw),math.sin(yaw)
    R = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return (SIGNS * (box[3:6]/2)) @ R.T + box[:3], R


def make_lidar(raw_points, box, corners, box_rotation):
    delta = raw_points - box[:3]
    local = delta @ box_rotation
    half = box[3:6]/2
    # Include nearby returns lying outside the prediction. These are not instance-segmented.
    keep = (np.abs(local) <= half + [1.5,1.5,1.0]).all(axis=1)
    local = local[keep]
    delta = delta[keep]
    canvas = Image.new('RGB',(960,525),BG)
    draw = ImageDraw.Draw(canvas)
    draw.text((12,6),'LIDAR_TOP  |  current box (orange)  |  nearby raw returns (white)',fill=TEXT,font=font(17))
    draw.text((12,34),'TOP   reference LiDAR: +x right, +y up',fill=HEAD,font=font(15))
    draw.text((493,34),'SIDE   box-local +length right, +z up',fill=HEAD,font=font(15))
    draw.text((493,275),'END   box-local +width right, +z up',fill=HEAD,font=font(15))

    def panel(rect, cloud, footprint, extents, front=None):
        left,top,right,bottom = rect
        centre = np.array([(left+right)/2,(top+bottom)/2])
        scale = min((right-left-28)/(2*extents[0]),(bottom-top-28)/(2*extents[1]))
        basis = np.array([scale,-scale])
        pts = np.rint(cloud*basis+centre).astype(int)
        for px,py in pts:
            if left+1 <= px < right-1 and top+1 <= py < bottom-1:
                draw.ellipse((px-1,py-1,px+1,py+1),fill=TEXT)
        polygon = [tuple(p) for p in np.rint(footprint*basis+centre).astype(int).tolist()]
        draw.line(polygon+[polygon[0]],fill=ORANGE,width=2)
        if front is not None:
            a,b = np.asarray(front)*basis+centre
            draw.line([tuple(a),tuple(b)],fill=ORANGE,width=3)
        metres = next((v for v in (0.1,0.25,0.5,1,2,5,10,20) if scale*v>=49),20.)
        x,y = left+8,bottom-7
        draw.line([(x,y),(x+metres*scale,y)],fill=HEAD,width=2)
        draw.text((x,y-18),f'{metres:g} m',fill=HEAD,font=font(13))

    dims = half
    hull = (corners[:4,:2]-box[:2])
    panel((6,57,476,510),delta[:,:2],hull,np.max(abs(hull),axis=0)+1.65,
          [[0,0],(box_rotation[:2,0]*dims[0]*0.75).tolist()])
    for axis,rect in [(0,(489,57,954,259)),(1,(489,297,954,510))]:
        other=1-axis
        slab=local[abs(local[:,other])<=dims[other]+.35]
        poly=np.array([[-dims[axis],-dims[2]],[dims[axis],-dims[2]],
                       [dims[axis],dims[2]],[-dims[axis],dims[2]]])
        panel(rect,slab[:,[axis,2]],poly,np.array([dims[axis]+1.65,dims[2]+1.15]))
    return canvas, {'nearby_returns':int(len(local)), 'inside_current_box':int((np.abs(local)<=half).all(axis=1).sum())}


def clip_to_image(a,b,width,height):
    """Parametric clipping to the image before drawing or computing a crop."""
    d=b-a
    lo,hi=0.,1.
    for coord,upper in [(0,width-1),(1,height-1)]:
        if abs(d[coord])<1e-12:
            if not 0<=a[coord]<=upper:
                return None
            continue
        t0,t1=sorted((-a[coord]/d[coord],(upper-a[coord])/d[coord]))
        lo,hi=max(lo,t0),min(hi,t1)
        if hi<lo:
            return None
    return a+lo*d,a+hi*d


def projected_edges(corners,K,width,height):
    lines=[]
    for i,j in EDGES:
        a,b=corners[i].copy(),corners[j].copy()
        if max(a[2],b[2])<=.10:
            continue
        if a[2]<.10:
            a+=(b-a)*((.10-a[2])/(b[2]-a[2]))
        if b[2]<.10:
            b+=(a-b)*((.10-b[2])/(a[2]-b[2]))
        q=np.array([a,b]) @ K.T
        uv=q[:,:2]/q[:,2,None]
        visible=clip_to_image(*uv,width,height)
        if visible is not None:
            lines.append(visible)
    return lines


def camera_view(source, world_corners, cam, pose, cal, name):
    corners=to_child(to_child(world_corners,pose),cal)
    with Image.open(source/cam['path']) as photo:
        w,h=photo.size
        edges=projected_edges(corners,np.asarray(cal['camera_intrinsic']),w,h)
        if edges:
            pts=np.array(edges).reshape(-1,2)
            a,b=pts.min(axis=0),pts.max(axis=0)
            pad=max(40.,max(b-a)*.55)
            crop=(max(0,math.floor(a[0]-pad)), max(0,math.floor(a[1]-pad)),
                  min(w,math.ceil(b[0]+pad+1)),min(h,math.ceil(b[1]+pad+1)))
            image=photo.crop(crop).convert('RGB')
            scale=min(608/image.width,420/image.height,2.)
            new_size=(max(1,round(image.width*scale)),max(1,round(image.height*scale)))
            factor=np.array(new_size)/np.array(image.size)
            image=image.resize(new_size,Image.Resampling.LANCZOS)
            drawing=ImageDraw.Draw(image)
            for p,q in edges:
                drawing.line([tuple((p-crop[:2])*factor),tuple((q-crop[:2])*factor)],fill=ORANGE,width=2)
        else:
            # JPEG decoder can directly downsample when the full image has no projected box.
            photo.draft('RGB',(400,225))
            image=photo.convert('RGB')
            image.thumbnail((432,285),Image.Resampling.LANCZOS)
    width=max(image.width,280)
    panel=Image.new('RGB',(width,image.height+28),BG)
    panel.paste(image,((width-image.width)//2,28))
    ImageDraw.Draw(panel).text((6,4),name+('  |  current box' if edges else '  |  no box projection'),fill=TEXT,font=font(13))
    return panel,bool(edges)


def save_jpeg(image,path):
    with tempfile.NamedTemporaryFile(dir=path.parent,delete=False) as f:
        temp=Path(f.name)
        try:
            image.save(f,format='JPEG',quality=89,subsampling=0)
        except BaseException:
            temp.unlink(missing_ok=True)
            raise
    os.replace(temp,path)


def official_context(raw, tasks):
    """Join sample → keyframe sample_data → sensor, calibration and ego pose."""
    meta = raw / 'v1.0-trainval'
    tokens = {t['sample_token'] for t in tasks}
    frames = lookup(meta / 'sample.json', tokens)
    scenes = lookup(meta / 'scene.json', {f['scene_token'] for f in frames.values()})
    sensors = {r['token']: r['channel'] for r in json.loads((meta/'sensor.json').read_text())}
    calibrations = {r['token']: r for r in json.loads((meta/'calibrated_sensor.json').read_text())}
    wanted = {'LIDAR_TOP', *CAMERAS}
    streams = {token: {} for token in tokens}
    with (meta / 'sample_data.json').open('rb') as stream:
        for row in ijson.items(stream, 'item', use_float=True):
            token = row['sample_token']
            if token not in streams or not row['is_key_frame']:
                continue
            cal = calibrations[row['calibrated_sensor_token']]
            channel = sensors[cal['sensor_token']]
            if channel in wanted:
                if channel in streams[token]:
                    raise ValueError(f'Duplicate {channel} keyframe in {token}')
                streams[token][channel] = row
    for token, channels in streams.items():
        if channels.keys() != wanted:
            raise ValueError(f'Missing channels in {token}: {wanted - channels.keys()}')
    poses = lookup(meta / 'ego_pose.json', {r['ego_pose_token'] for channels in streams.values() for r in channels.values()})
    result = {}
    for token, channels in streams.items():
        lidar = channels['LIDAR_TOP']
        scene_token = frames[token]['scene_token']
        cameras = {name: dict(path=channels[name]['filename'], timestamp=channels[name]['timestamp'],
                              calibrated_sensor_token=channels[name]['calibrated_sensor_token'],
                              ego_pose_token=channels[name]['ego_pose_token']) for name in CAMERAS}
        result[token] = dict(scene_token=scene_token, scene_name=scenes[scene_token]['name'],
                             timestamp=lidar['timestamp'], reference_sample_data_token=lidar['token'],
                             pointcloud_path=lidar['filename'], ego_pose=poses[lidar['ego_pose_token']],
                             calibrated_sensor=calibrations[lidar['calibrated_sensor_token']], cameras=cameras)
    return result, poses, calibrations


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tasks',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--dataroot',type=Path,required=True, help='Directory containing samples/ and v1.0-trainval/')
    p.add_argument('--workers',type=int,default=4)
    p.add_argument('--limit',type=int)
    args=p.parse_args()
    if args.workers<1 or (args.limit is not None and args.limit<1):
        p.error('workers and limit must be positive')
    tasks=list(jsonl(args.tasks))
    if args.limit is not None:
        tasks=tasks[:args.limit]
    ids=[t['id'] for t in tasks]
    if not ids or len(set(ids))!=len(ids) or any(Path(i).name!=i or i in ('.','..','') for i in ids):
        p.error('expected unique, nonempty directory-safe action IDs')
    raw=args.dataroot.resolve()
    samples, camera_poses, camera_calibs = official_context(raw, tasks)
    args.out.mkdir(parents=True,exist_ok=True)
    version=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    def render(t):
        s=samples[t['sample_token']]
        assert s['scene_token']==t['scene_token']
        if 'reference_sample_data_token' in t:
            assert s['reference_sample_data_token']==t['reference_sample_data_token']
        source=[version,t['id'],t['episode_id'],t['state_id'],t['sample_token'],t['box']]
        key=hashlib.sha256(json_bytes(source)).hexdigest()
        folder=args.out/'views'/t['id']
        folder.mkdir(parents=True,exist_ok=True)
        paths=[folder/(view+'.jpg') for view in VIEWS]
        marker=folder/'view.json'
        if marker.exists():
            old=json.loads(marker.read_bytes())
            if old['render_sha256']==key and all(path.is_file() and path.stat().st_size>0 for path in paths):
                return old
            raise ValueError(f"Existing observation has a different renderer/state or incomplete assets: {t['id']}")
        box=np.asarray(t['box'],dtype=float)
        corners,R=box_geometry(box)
        cloud=np.fromfile(raw/s['pointcloud_path'],dtype='<f4').reshape(-1,5)[:,:3]
        figure,stats=make_lidar(cloud,box,corners,R)
        save_jpeg(figure,paths[0])
        wc=to_parent(to_parent(corners,s['calibrated_sensor']),s['ego_pose'])
        projected=[]
        for name,path in zip(CAMERAS,paths[1:]):
            c=s['cameras'][name]
            image,hit=camera_view(raw,wc,c,camera_poses[c['ego_pose_token']],
                                  camera_calibs[c['calibrated_sensor_token']],name)
            save_jpeg(image,path)
            if hit:
                projected.append(name)
        record=dict(id=t['id'],episode_id=t['episode_id'],state_id=t['state_id'],sample_token=t['sample_token'],
                    scene_token=t['scene_token'],scene=s['scene_name'],category=t['category'],
                    images=[str(path.relative_to(args.out)) for path in paths],projected=projected,
                    lidar_stats=stats,lidar_timestamp=s['timestamp'],
                    camera_offsets_us={k:s['cameras'][k]['timestamp']-s['timestamp'] for k in CAMERAS},
                    render_sha256=key,renderer_sha256=version)
        write_atomic(marker,json_bytes(record))
        return record
    records={}
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(render,t) for t in tasks]
        for f in as_completed(futures):
            r=f.result()
            records[r['id']]=r
            if len(records)%50==0:
                print(f'{len(records)}/{len(tasks)} rendered',flush=True)
    full_manifest = args.out/'observations.jsonl'
    if full_manifest.exists():
        reference = {r['id']:r for r in jsonl(full_manifest)}
        for key, rec in records.items():
            for field in ('episode_id','state_id','sample_token','scene_token','category','images','projected',
                          'lidar_stats','lidar_timestamp','camera_offsets_us'):
                if rec[field] != reference[key][field]:
                    raise ValueError(f'{key}: rendered observation differs from original {field}')
    else:
        name = 'observations.partial.jsonl' if args.limit is not None else 'observations.jsonl'
        write_atomic(args.out/name,b''.join(json_bytes(records[i]) for i in ids))
    print(f'{len(records)} seven-view records, {sum(bool(r["projected"]) for r in records.values())} with camera projection',flush=True)


if __name__=='__main__':
    main()
