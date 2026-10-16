#!/usr/bin/env python3
"""状态-观测自洽率：把 tracker 日志里的 ctr/r 与实测板位对齐，先拟合帧偏置再算误差。

为什么必须拟合帧偏置：/imu/rpy 是原始 yaw，tracker 内部用的是**开机归零后**的 yaw，
两者差一个绕世界 Z 的旋转（实测量级 40~50°），直接算距离会得到跨坐标系的假结果。

用法: python3 compare_runs.py <csv前缀> [tracker日志路径]
输出: 帧偏置、| |ctr-板| - r | 的 p50/p95、"已锚定(<0.1m)"占比、r 分布
判据: p50 应 ≈0（<0.05 为佳）；"已锚定"占比越高越好；r 应稳定在表值(如 0.30/0.27)，贴 clamp 说明 r 不可观问题还在
"""
import sys, re, csv, math, glob, os, bisect
import numpy as np

PREFIX = sys.argv[1] if len(sys.argv) > 1 else "probe"
LOG = sys.argv[2] if len(sys.argv) > 2 else None
if LOG is None:
    logs = sorted(glob.glob(os.path.expanduser("~/.ros/log/rm_tracker_node_*.log")), key=os.path.getmtime)
    LOG = logs[-1] if logs else None

def Rx(a): c,s=math.cos(a),math.sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(a): c,s=math.cos(a),math.sin(a); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(a): c,s=math.cos(a),math.sin(a); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def euler(rpy): r,p,y=[math.radians(v) for v in rpy]; return Rz(y)@Ry(p)@Rx(r)
CAM_FIX=(-90.0,0.0,-90.0)
def cam_to_world(x,rpy): return euler(rpy)@(euler(CAM_FIX)@np.asarray(x,float))

S=[]
for line in open(LOG, errors="ignore"):
    m=re.search(r"\[(\d+\.\d+)\].*spin: (\d+) vyaw: (-?[\d.]+) \| ctr: \((-?[\d.]+), (-?[\d.]+)\) r: ([\d.]+)", re.sub(r"\x1b\[[0-9;]*m","",line))
    if m: S.append(tuple(float(m.group(i)) for i in range(1,7)))
print("tracker 日志:", os.path.basename(LOG), " ctr 采样数:", len(S))
if not S: sys.exit("日志里没有 ctr/r 字段——需要 tracker 侧那行 1Hz 诊断日志")

def load(p): return [r for r in csv.DictReader(open(p)) if r.get("stamp")]
A=load(PREFIX+"armors.csv"); I=load(PREFIX+"imu.csv")
AT=np.array([float(r["stamp"]) for r in A]); IT=np.array([float(r["stamp"]) for r in I])
def near(T,t,tol):
    i=bisect.bisect_left(T,t); b=None; bd=1e9
    for j in (i-1,i,i+1):
        if 0<=j<len(T) and abs(T[j]-t)<bd: bd=abs(T[j]-t); b=j
    return b if bd<=tol else None
rows=[]
for t,spin,vyaw,cx,cy,r in S:
    if not (AT[0]<=t<=AT[-1]): continue
    j=near(AT,t,0.06); 
    if j is None: continue
    a=A[j]; k=near(IT,float(a["stamp"]),0.05)
    if k is None: continue
    rpy=(float(I[k]["roll"]),float(I[k]["pitch"]),float(I[k]["yaw"]))
    pw=cam_to_world(np.array([float(a["dx"]),float(a["dy"]),float(a["dz"])])/1000.0, rpy)
    rows.append((cx,cy,r,pw[0],pw[1],spin,vyaw))
if len(rows)<5: sys.exit("成对样本太少(%d)，确认采集中 tracker 有在跟" % len(rows))
X=np.array(rows)
best=None
for deg in np.arange(-180,180,0.5):
    th=math.radians(deg); c,s=math.cos(th),math.sin(th)
    d=np.hypot(X[:,0]-(c*X[:,3]-s*X[:,4]), X[:,1]-(s*X[:,3]+c*X[:,4]))
    sc=np.median(np.abs(d-X[:,2]))
    if best is None or sc<best[0]: best=(sc,deg,d)
res,deg,d=best
print("拟合帧偏置 deltaYaw = %+.1f deg" % deg)
print("| |ctr-板| - r |   p50=%.3f m  p95=%.3f m   (应≈0；>0.3 说明状态与观测脱节)" % (res, np.percentile(np.abs(d-X[:,2]),95)))
print("已锚定(|差|<0.1m) 占比 = %.0f%%" % (100*np.mean(np.abs(d-X[:,2])<0.1)))
print("r: p50=%.3f p5=%.3f p95=%.3f ; 贴 0.40 clamp 占比=%.0f%%" % (
    np.median(X[:,2]), np.percentile(X[:,2],5), np.percentile(X[:,2],95), 100*np.mean(X[:,2]>0.395)))
print("spin=1 占比=%.0f%% ; |vyaw| p95=%.2f rad/s" % (100*np.mean(X[:,5]>0), np.percentile(np.abs(X[:,6]),95)))
