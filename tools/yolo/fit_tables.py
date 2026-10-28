"""Fit measured-size -> true-size / pixel-error tables from calib/before.csv; write eval/sensing/yoloErrorModel.m (stage 1: k=1)."""
import numpy as np, pandas as pd, json, sys
d=pd.read_csv('calib/before.csv'); d=d[(d.method=='fuse')&(d.label!='fp')]
mad=lambda x:1.4826*np.median(np.abs(x-np.median(x)))
def tab(g, s, ts, vq, tvq, edges, nmin=40):
    g=g.copy(); g['b']=pd.cut(g[s],edges); rows=[]
    for b,x in g.groupby('b',observed=True):
        if len(x)<nmin: continue
        rows.append(dict(n=len(x),s=np.median(x[s]),tm=np.median(x[ts]),tsd=mad(x[ts]-x[ts].median()) if False else mad(x[ts]-x[s]*np.median(x[ts]/x[s])),
            vb=np.median(x[vq]-x[tvq]),vsd=mad(x[vq]-x[tvq]),usd=mad(x.u-x.tu)))
    t=pd.DataFrame(rows)
    for c in ['tsd','vsd','usd']:
        v=t[c].values; sm=v.copy()
        for i in range(len(v)): sm[i]=np.mean(v[max(0,i-1):i+2])
        t[c]=sm
    t['tm']=np.maximum.accumulate(t.tm.values)
    return t
b=d[(d.cls=='ball')&(d.conf>=0.6)]
tb=tab(b,'w','tw','v','tv',[0,6.5,7.5,8.5,10,12,14,17,20,26,400])
r=d[(d.cls=='robot')&(d.conf>=0.2)].copy(); r['vf']=r.v+r.h/2; r['tvf']=r.tv+r.th/2
r=r[r.h>=23]
tr=tab(r,'h','th','vf','tvf',[0,23,26,30,36,44,55,70,90,120,400])
print(tb.round(3)); print(tr.round(3))
tb.to_csv('calib/tab_ball.csv',index=False); tr.to_csv('calib/tab_robot.csv',index=False)
