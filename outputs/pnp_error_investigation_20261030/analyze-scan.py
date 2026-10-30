"""Offline pose/feature scoring against separately recorded simulation truth."""
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
NOMINAL = np.array([[-.0675,-.029,0],[-.0675,.029,0],[.0675,-.029,0],[.0675,.029,0]])


def read_lines(path: Path) -> list[dict]:
    with path.open(encoding="utf-8-sig") as stream:
        return [json.loads(line) for line in stream]


def project(points: np.ndarray, sensor: dict) -> np.ndarray:
    rotation = np.array(sensor["optical_rotation"])
    origin = np.array(sensor["optical_transform"])[:3,3]
    camera = (points-origin)@rotation
    x,y = (camera[:,:2]/camera[:,2:]).T
    k1,k2,p1,p2,k3 = sensor["D"]
    r2 = x*x+y*y; scale = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
    distorted = np.array([x*scale+2*p1*x*y+p2*(r2+2*x*x),y*scale+p1*(r2+2*y*y)+2*p2*x*y]).T
    k = np.array(sensor["K"])
    return distorted@k[:2,:2].T+k[:2,2]


def board_geometry(armor: dict, center: np.ndarray) -> tuple[np.ndarray,np.ndarray]:
    point = np.array(armor["position_world_m"])
    radial = point-center; radial[2]=0; radial/=np.linalg.norm(radial)
    normal = np.cos(np.pi/12)*radial+np.array([0,0,np.sin(np.pi/12)])
    basis = np.column_stack((np.cross([0,0,1],radial),-np.sin(np.pi/12)*radial+np.array([0,0,np.cos(np.pi/12)]),normal))
    return point,basis


def stats(values: list[float]) -> dict:
    a=np.array([v for v in values if v is not None and np.isfinite(v)])
    if not len(a):return {"n":0}
    return dict(n=len(a),mean=float(a.mean()),std=float(a.std()),rmse=float(np.sqrt(np.mean(a*a))),p95_abs=float(np.percentile(abs(a),95)))


def analyze(run: Path) -> tuple[list[dict],dict,list[dict]]:
    plan=json.loads((run/"scenario.json").read_text(encoding="utf-8-sig"))
    robots=json.loads((run/"robots.json").read_text(encoding="utf-8-sig"))
    distance=robots[1]["position_m"][1]-robots[0]["position_m"][1]
    yaw=robots[1]["yaw_deg"]
    frames=read_lines(run/"vision/frames.jsonl")
    saved=list((run/"vision").glob("frame_*/image_raw.png"))
    snapshot_time=float(saved[0].parent.name.rsplit("_t",1)[1]) if saved else 2.6
    truth=read_lines(run/"session/cameras/0/truth.jsonl"); tt=np.array([r["stamp"]["time_s"] for r in truth])
    rows=[]; center_errors=[]; probes=[]; snapshots={}
    for f in frames:
        if f["simulation_time_s"]<1.5:continue
        i=int(np.argmin(abs(tt-f["simulation_time_s"])))
        if abs(tt[i]-f["simulation_time_s"])>.02:continue
        tr=truth[i]; center=np.array(tr["centre_world_m"]);sensor=f["input"]
        rotation=np.array(sensor["optical_rotation"]);origin=np.array(sensor["optical_transform"])[:3,3]
        boards=[]
        for armor in tr["armors"]:
            point,basis=board_geometry(armor,center)
            if basis[:,2]@(origin-point)<=0:continue
            ideal=project(NOMINAL@basis.T+point,sensor)
            if ideal[:2,0].mean()>ideal[2:,0].mean():ideal=ideal[[2,3,0,1]]
            boards.append((armor["side"],point,basis,ideal))
        debug=f["tracker_debug"]
        active=f["state"] in ("TRACKING","TEMP_LOST")
        state=np.array(debug["body_ekf_state"])
        if active:center_errors.append(float(np.linalg.norm((origin+state[[0,2,4]]-center)[:2])))
        models={}
        if f["state"]=="TRACKING":
            for model in debug["body_plates"]:
                if not model["matched"]:continue
                point=origin+rotation@np.mean(model["corners_camera_m"],axis=0)
                pixel=project(point[None,:],sensor)[0]
                b=min(boards,key=lambda b:np.linalg.norm(project(b[1][None,:],sensor)[0]-pixel))
                models[b[0]]=point
        for obs in f["observations"]:
            if obs["armor_id"]!=2:continue
            pixels=np.array(obs["corners_px"])
            b=min(boards,key=lambda b:np.linalg.norm(b[3].mean(0)-pixels.mean(0)))
            side,point,basis,ideal=b
            if np.linalg.norm(ideal.mean(0)-pixels.mean(0))>12:continue
            measured=origin+rotation@np.array(obs["position_camera_m"])
            ray=(point-origin)/np.linalg.norm(point-origin)
            lateral=np.cross([0,0,1],ray);lateral/=np.linalg.norm(lateral)
            error=measured-point;normal=rotation@np.array(obs["normal_camera"])
            model_error=models[side]-point if side in models else None
            row=dict(run=run.relative_to(ROOT).as_posix(),distance_m=distance,yaw_deg=yaw,side=side,frame=f["frame_number"],time_s=f["simulation_time_s"],
                range_m=float(np.linalg.norm(point-origin)),incidence_deg=float(np.degrees(np.arccos(np.clip(-basis[:,2]@ray,-1,1)))),
                depth_error_mm=float(error@ray)*1000,lateral_error_mm=float(error@lateral)*1000,vertical_error_mm=float(error[2])*1000,
                normal_error_deg=float(np.degrees(np.arccos(np.clip(normal@basis[:,2],-1,1)))),
                reprojection_px=obs["pnp"]["reprojection_error_px"],
                endpoint_rms_px=float(np.sqrt(np.mean(np.sum((pixels-ideal)**2,axis=1)))),
                left_length_ratio=float(np.linalg.norm(pixels[1]-pixels[0])/np.linalg.norm(ideal[1]-ideal[0])),
                right_length_ratio=float(np.linalg.norm(pixels[3]-pixels[2])/np.linalg.norm(ideal[3]-ideal[2])),
                width_ratio=float(np.linalg.norm(pixels[:2].mean(0)-pixels[2:].mean(0))/np.linalg.norm(ideal[:2].mean(0)-ideal[2:].mean(0))),
                ekf_depth_error_mm=float(model_error@ray)*1000 if model_error is not None else None,
                ekf_lateral_error_mm=float(model_error@lateral)*1000 if model_error is not None else None,
                ekf_vertical_error_mm=float(model_error[2])*1000 if model_error is not None else None)
            rows.append(row)
            row['paired_pnp_depth_error_mm']=row['depth_error_mm'] if model_error is not None else None
            if side not in snapshots or abs(f["simulation_time_s"]-snapshot_time)<abs(snapshots[side]["time"]-snapshot_time):
                snapshots[side]=dict(time=f["simulation_time_s"],input=sensor,corners=pixels.tolist(),ideal=ideal.tolist(),position_camera=((point-origin)@rotation).tolist(),normal_camera=(basis[:,2]@rotation).tolist(),side=side,center_world=point.tolist(),basis=basis.tolist())
    for side,snap in snapshots.items():probes.append(dict(run=run.relative_to(ROOT).as_posix(),**snap))
    summary=dict(run=run.relative_to(ROOT).as_posix(),distance_m=distance,yaw_deg=yaw,frames=len(frames),observations=len(rows),center_error_mm=stats([v*1000 for v in center_errors]),sides={})
    keys=[k for k in rows[0] if (k.endswith(("_mm","_deg","_px","_ratio")) or k=="range_m") and k!="yaw_deg"] if rows else []
    for side in sorted({r["side"] for r in rows}):
        selected=[r for r in rows if r["side"]==side]
        summary["sides"][side]={k:stats([r[k] for r in selected]) for k in keys}
    return rows,summary,probes


if __name__=="__main__":
    all_rows=[];summary=[];probes=[]
    for run in sorted(ROOT.glob("scan-*/*/vision/report.json")):
        r,s,p=analyze(run.parent.parent);all_rows+=r;summary.append(s);probes+=p
        print(s["run"],{side:{k:round(v[k]["mean"],2) for k in ("depth_error_mm","normal_error_deg","ekf_depth_error_mm") if v[k]["n"]} for side,v in s["sides"].items()},flush=True)
    with (ROOT/"observations.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=all_rows[0]);writer.writeheader();writer.writerows(all_rows)
    (ROOT/"scan-summary.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    (ROOT/"pose-probes.jsonl").write_text("".join(json.dumps(p)+"\n" for p in probes),encoding="utf-8")
