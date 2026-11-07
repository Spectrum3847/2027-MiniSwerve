"""Fits the sim's timing and power from real SystemCore match logs: main loop cycle time, CAN sample
spacing and failed reads, camera latency, and battery open-circuit voltage over a match.

    python tools/fit_timing.py [<log.wpilog> ...]   (default: FM's TXMAN1 matches)

Uses each log's first 155 s enabled. Keys are FM 2026's (LoggedRobot/FullCycleMS,
Swerve/OdometryTimestamps, Swerve/FailedDaqs and SuccessfulDaqs, Vision/Orin/orin-left/LatencyMs,
BatteryLogger/BatteryVoltage and Current).
"""
import sys,glob,math,statistics; sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from wpilog_read import read_wpilog, series
logs=sys.argv[1:] or sorted(glob.glob("../2026-FM-SystemCore/logs/txman1-ns/*_q*_ns.wpilog"))
cyc=[]; per=[]; fails=[]; succ=[]; vb=[]; lat=[]
for p in logs:
    r,t=read_wpilog(p)
    en=series(r,t,"/DriverStation/Enabled"); on=[a for a,v in en if v]
    if not on: continue
    t0=on[0]; t1=t0+155
    cyc+= [v for a,v in series(r,t,"/RealOutputs/LoggedRobot/FullCycleMS") if t0<=a<=t1 and v is not None]
    ts=[v for a,v in series(r,t,"/Swerve/OdometryTimestamps") if t0<=a<=t1 and v]
    flat=[x for arr in ts for x in arr]
    per+= [b-a for a,b in zip(flat,flat[1:]) if 0<b-a<0.1]
    f=series(r,t,"/Swerve/FailedDaqs"); s=series(r,t,"/Swerve/SuccessfulDaqs")
    fv=[v for a,v in f if t0<=a<=t1]; sv=[v for a,v in s if t0<=a<=t1]
    if fv and sv: fails.append(fv[-1]-fv[0]); succ.append(sv[-1]-sv[0])
    v=[(a-t0,x) for a,x in series(r,t,"/RealOutputs/BatteryLogger/BatteryVoltage") if t0<=a<=t1]
    i=[(a-t0,x) for a,x in series(r,t,"/RealOutputs/BatteryLogger/Current") if t0<=a<=t1]
    vb.append((p[-20:],v,i))
    lat+= [x for a,x in series(r,t,"/RealOutputs/Vision/Orin/orin-left/LatencyMs") if t0<=a<=t1 and x is not None]
def pct(a,q): a=sorted(a); return a[min(len(a)-1,int(q*len(a)))]
print("LOOP FullCycleMS n",len(cyc)," p50 %.1f p90 %.1f p99 %.1f max %.1f  share>20 %.2f share>40 %.3f"%(pct(cyc,.5),pct(cyc,.9),pct(cyc,.99),max(cyc),sum(c>20 for c in cyc)/len(cyc),sum(c>40 for c in cyc)/len(cyc)))
print("ODOM period n",len(per)," p50 %.2f p10 %.2f p90 %.2f p99 %.2f ms"%(pct(per,.5)*1e3,pct(per,.1)*1e3,pct(per,.9)*1e3,pct(per,.99)*1e3))
print("DAQ failed",sum(fails),"of",sum(fails)+sum(succ),"= %.4f"%(sum(fails)/max(1,sum(fails)+sum(succ))))
print("CAM latency ms n",len(lat)," p50 %.1f p90 %.1f"%(pct(lat,.5),pct(lat,.9)) if lat else "no latency")
for name,v,i in vb:
    if len(v)<50: continue
    # open-circuit estimate: V + R*I with R=0.012 in early 10 s vs last 10 s
    def voc(win):
        pts=[x for a,x in v if win[0]<=a<win[1]]; cur=[x for a,x in i if win[0]<=a<win[1]]
        return statistics.mean(pts)+0.012*statistics.mean(cur) if pts and cur else float('nan')
    ah=sum((b[0]-a[0])*(a[1]+b[1])/2 for a,b in zip(i,i[1:]) if 0<b[0]-a[0]<1)/3600 if len(i)>1 else 0
    print("BATT",name,"Voc start %.2f end %.2f  drop %.2f V  charge used %.2f Ah(est)"%(voc((0,10)),voc((145,155)),voc((0,10))-voc((145,155)),ah))
import collections
h=collections.Counter(round(p*1e3) for p in per); tot=sum(h.values())
print("ODOM period histogram (ms: share):",{k:round(v/tot,4) for k,v in sorted(h.items()) if v/tot>0.0005})
print("LOOP >5ms %.4f >10ms %.5f >20ms %.6f; count>10 per match %.1f"%(sum(c>5 for c in cyc)/len(cyc),sum(c>10 for c in cyc)/len(cyc),sum(c>20 for c in cyc)/len(cyc),sum(c>10 for c in cyc)/max(1,len(vb))))
lh=collections.Counter(int(x//2)*2 for x in lat); lt=sum(lh.values()); print("CAM latency hist:",{k:round(v/lt,3) for k,v in sorted(lh.items()) if v/lt>0.005})
