"""Offline mesh visibility audit; does not change or feed the online detector.

Sample AM02 light-cap triangles at <= 0.6 mm edge spacing, deduplicate within
0.25 mm grid cells, and retain the original mesh vertices. Cast segments
from the recorded camera to those points through the actual medium LOD meshes.
Compare cap silhouette with/without the front/back plate and number sticker.
The silhouette is geometric, not a prediction of thresholded/shaded pixels.
"""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from importlib.util import spec_from_file_location, module_from_spec

ROOT=Path(__file__).resolve().parent
SIM=ROOT.parents[2]
spec=spec_from_file_location("scan",ROOT/"analyze-scan.py");scan=module_from_spec(spec);spec.loader.exec_module(scan)

def mesh(name):
    vertices=[];faces=[]
    for line in (SIM/f"assets/robots/model_levels/medium/{name}.obj").read_text().splitlines():
        fields=line.split()
        if fields and fields[0]=="v":vertices.append(list(map(float,fields[1:4])))
        if fields and fields[0]=="f":
            ids=[int(f.split('/')[0])-1 for f in fields[1:]]
            faces.extend([[ids[0],ids[i],ids[i+1]] for i in range(1,len(ids)-1)])
    return np.array(vertices)[faces]

def samples(triangles):
    result=[]
    for t in triangles:
        n=max(1,int(np.ceil(np.linalg.norm(np.roll(t,-1,axis=0)-t,axis=1).max()/.0006)))
        a,b=np.array([(i/n,j/n) for i in range(n+1) for j in range(n+1-i)]).T
        result.append(t[0]+a[:,None]*(t[1]-t[0])+b[:,None]*(t[2]-t[0]))
    points=np.concatenate(result)
    _,ids=np.unique(np.round(points/.00025).astype(int),axis=0,return_index=True)
    return np.unique(np.round(np.concatenate((points[ids],triangles.reshape(-1,3))),10),axis=0)

def visible(points,origin,triangles):
    e1=triangles[:,1]-triangles[:,0];e2=triangles[:,2]-triangles[:,0]
    offset=origin-triangles[:,0];q=np.cross(offset,e1);numer=np.sum(e2*q,axis=1)
    result=[]
    for start in range(0,len(points),128):
        rays=points[start:start+128]-origin
        h=np.cross(rays[:,None,:],e2[None,:,:]);det=np.sum(h*e1,axis=2)
        inverse=np.divide(1.,det,out=np.zeros_like(det),where=abs(det)>1e-14)
        u=np.sum(offset*h,axis=2)*inverse
        v=np.einsum('pi,ti->pt',rays,q)*inverse;t=numer*inverse
        blocked=(abs(det)>1e-14)&(u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)&(t>0)&(t<1-1e-7)
        result.extend(~blocked.any(axis=1))
    return np.array(result)

def hull(points):
    points=sorted(set(map(tuple,points)));lower=[];upper=[]
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    for target,order in ((lower,points),(upper,reversed(points))):
        for p in order:
            while len(target)>1 and cross(target[-2],target[-1],p)<=0:target.pop()
            target.append(p)
    return np.array(lower[:-1]+upper[:-1])

def rectangle(points):
    polygon=hull(points);best=None
    for edge in np.roll(polygon,-1,axis=0)-polygon:
        u=edge/np.linalg.norm(edge);axes=np.column_stack((u,[-u[1],u[0]]))
        local=polygon@axes;low=local.min(0);high=local.max(0);extent=high-low
        if best is None or np.prod(extent)<best[0]:
            along=np.argmax(extent);ends=np.tile((low+high)/2,(2,1));ends[:,along]=[low[along],high[along]]
            best=(np.prod(extent),ends@axes.T)
    return best[1][np.argsort(-best[1][:,1])]

if __name__=="__main__":
    caps=[mesh(f"am02_light_{side}_cap") for side in ('l','r')]
    cap_points=[samples(c) for c in caps]
    support=[mesh(n) for n in ('am02_part_01','am02_light_l_mount','am02_light_r_mount')]
    cover=[mesh(n) for n in ('am02_front_plate_front','am02_front_plate_back','am02_flat_sticker')]
    uncovered=np.concatenate(caps+support);covered=np.concatenate(caps+support+cover)
    result=[];mesh_probes=[]
    for probe in scan.read_lines(ROOT/"pose-probes.jsonl"):
        # The near cases make the physical occlusion distinguishable from pixel sampling.
        if not any(tag in probe['run'] for tag in ('/d1.4-y','/d5-y0')):continue
        basis=np.array(probe['basis']);center=np.array(probe['center_world']);sensor=probe['input']
        origin=np.array(sensor['optical_transform'])[:3,3]
        local_origin=(origin-center)@basis+np.array([0,0,.0002])
        entry=dict(run=probe['run'],side=probe['side'],camera_in_mesh_m=local_origin.tolist(),lights=[])
        for points in cap_points:
            with_cover=visible(points,local_origin,covered);without_cover=visible(points,local_origin,uncovered)
            def px(mask):return scan.project(points[mask]@basis.T+center-.0002*basis[:,2],sensor)
            full=px(without_cover);visible_px=px(with_cover)
            a=rectangle(full);b=rectangle(visible_px)
            entry['lights'].append(dict(uncovered_ends=a.tolist(),visible_ends=b.tolist(),height_without_cover_px=float(np.linalg.norm(a[1]-a[0])),height_with_cover_px=float(np.linalg.norm(b[1]-b[0])),mesh_y_minmax_mm=[float(points[with_cover,1].min()*1000),float(points[with_cover,1].max()*1000)],visible_hull=hull(visible_px).tolist()))
        entry['lights'].sort(key=lambda v:np.array(v['visible_ends'])[:,0].mean())
        entry['ideal']=probe['ideal'];entry['detected']=probe['corners']
        result.append(entry)
        mesh_probes.append(dict(probe,visible_mesh=[v for light in entry['lights'] for v in light['visible_ends']],uncovered_mesh=[v for light in entry['lights'] for v in light['uncovered_ends']]))
        print(probe['run'],[(round(l['height_without_cover_px'],2),round(l['height_with_cover_px'],2),[round(v,2) for v in l['mesh_y_minmax_mm']]) for l in entry['lights']],flush=True)
        images=list((ROOT/probe['run']/"vision").glob('frame_*/image_raw.png'))
        if '/d1.4-y30' in probe['run'] and images:
            canvas=Image.open(images[0]).convert('RGB');draw=ImageDraw.Draw(canvas)
            for light in entry['lights']:
                poly=[tuple(v) for v in light['visible_hull']];draw.line(poly+[poly[0]],fill='#44ff44',width=1)
            for points,color in ((entry['ideal'],'#ffce40'),(entry['detected'],'#ff55ff')):
                for x,y in points:draw.ellipse((x-2,y-2,x+2,y+2),outline=color,width=1)
            box=np.array(entry['ideal']);lo=np.floor(box.min(0)-[30,45]).astype(int);hi=np.ceil(box.max(0)+[30,30]).astype(int)
            canvas=canvas.crop((*lo,*hi)).resize(tuple((hi-lo)*4))
            ImageDraw.Draw(canvas).text((8,8),'Yellow: nominal 135 x 58 mm | Pink: detected | Green: visible mesh',fill='white')
            canvas.save(ROOT/'occlusion-overlay.png')
        for light in entry['lights']:light.pop('visible_hull')
    (ROOT/'occlusion-summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    (ROOT/'mesh-probes.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in mesh_probes),encoding='utf-8')
