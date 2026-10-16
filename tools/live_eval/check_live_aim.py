#!/usr/bin/env python3
"""命令-几何偏差：只用**相机系**量，无坐标系问题。

把 /tracker/gimbal_control 的 yaw/pitch 与由观测板位算出的几何期望比：
  期望 yaw  = atan2(dx, dz)
  期望 pitch= atan2(dy - 125mm, dz)     # +125mm 是相机->枪口的下移(cam_to_gun_pos_y)
注意: pitch 的 p50 会偏 -0.6~-0.8°，那是弹道下坠补偿（瞄点在目标上方），**不是 bug**；
      所以看 p95 与"帧间抖动"：抖动小但偏差大 => 模型位置(r/车心)偏，不是指令抖。

用法: python3 check_live_aim.py <csv前缀>
"""
import sys, csv, math, bisect
import numpy as np
PREFIX = sys.argv[1] if len(sys.argv) > 1 else "probe"
def load(p): return [r for r in csv.DictReader(open(p)) if r.get("stamp")]
A=load(PREFIX+"armors.csv"); G=load(PREFIX+"gimbal.csv")
AT=np.array([float(r["stamp"]) for r in A]); GT=np.array([float(r["stamp"]) for r in G])
def near(T,t,tol):
    i=bisect.bisect_left(T,t); b=None; bd=1e9
    for j in (i-1,i,i+1):
        if 0<=j<len(T) and abs(T[j]-t)<bd: bd=abs(T[j]-t); b=j
    return b if bd<=tol else None
errs=[]
for g in G[::max(1,len(G)//600)]:
    t=float(g["stamp"]); i=near(AT,t,0.02)
    if i is None: continue
    a=A[i]; dx,dy,dz=float(a["dx"]),float(a["dy"]),float(a["dz"])
    ey=math.degrees(math.atan2(dx,dz)); ep=math.degrees(math.atan2(dy-125.0,dz))
    errs.append((float(g["yaw"])-ey, float(g["pitch"])-ep))
E=np.array(errs)
if len(E)==0: sys.exit("没有可配对样本")
print("n=%d  命令-几何期望:  yaw p50=%+.2f  p95(|.|)=%.2f   pitch p50=%+.2f  p95(|.|)=%.2f deg" % (
    len(E), np.percentile(E[:,0],50), np.percentile(np.abs(E[:,0]),95),
    np.percentile(E[:,1],50), np.percentile(np.abs(E[:,1]),95)))
cy=np.array([float(g["yaw"]) for g in G]); cp=np.array([float(g["pitch"]) for g in G])
gt=np.array([float(g["stamp"]) for g in G]); m=np.diff(gt)<0.03
if m.sum():
    print("命令帧间抖动: yaw p95=%.2f  pitch p95=%.2f deg (抖动小+偏差大 => 模型位置偏)" % (
        np.percentile(np.abs(np.diff(cy))[m],95), np.percentile(np.abs(np.diff(cp))[m],95)))
print("基线(未修) : yaw p95 5.4 / pitch p95 4.1 ; 期望 ≲1")
