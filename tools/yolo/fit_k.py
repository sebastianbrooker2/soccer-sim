"""Fit heavy-tail range-std inflation k(est range), conf-group factors, LQ factor from a stage CSV (k=1 run); write calib/k.json."""
import numpy as np, pandas as pd, json, sys
src=sys.argv[1]
d=pd.read_csv(src); d=d[(d.method=='fuse')&(d.label!='fp')]
d['z']=(d.estR-d.trueR)/d.rstd
rk={'ball':[0,2,4,6,8,10,13],'robot':[0,2,4,6,8,10,13]}
out={}
q=lambda x:max(np.std(x),np.percentile(np.abs(x),99)/3)
edges=[0,2,4,6,8,10,13,100]
for cls,cmin,cedge in [('ball',0.6,[0,0.8,1.01]),('robot',0.2,[0,0.5,0.7,1.01])]:
    g=d[(d.cls==cls)&(d.conf>=cmin)].copy()
    g['grp']=np.clip(np.searchsorted(cedge,g.conf,side='right')-1,0,len(cedge)-2)
    top=g[g.grp==len(cedge)-2]
    ks=[];centers=[]
    for lo,hi in zip(edges[:-1],edges[1:]):
        x=top[(top.estR>=lo)&(top.estR<hi)]
        if len(x)<40: continue
        ks.append(q(x.z.values)); centers.append(float(np.median(x.estR)))
    ks=np.array(ks)
    ks=np.maximum(ks,1.0)
    if cls=="ball": ks=np.maximum.accumulate(ks)
    kfun=lambda r:np.interp(r,centers,ks)
    g['z1']=g.z/kfun(g.estR.values)
    nl=g[(g.lowq==0)|(cls=='robot')]
    cf=[1.0]*(len(cedge)-1)
    for gi in range(len(cedge)-1):
        x=nl[nl.grp==gi].z1.values
        cf[gi]=max(1.0,q(x)) if len(x)>30 else 1.0
    cfarr=np.array(cf)
    g['z2']=g.z1/cfarr[g.grp.values]
    lq=1.0
    if cls=='ball':
        l=g[g.lowq==1]; lq=max(1.0,q(l.z2.values))
    g['z3']=g.z2/(lq if cls=='ball' else 1)*0+g.z2/np.where(g.lowq==1,lq,1.0)
    print(cls,'k',dict(zip(np.round(centers,1),np.round(ks,2))),'conf',cedge,np.round(cfarr,2),'lq',round(lq,2),'-> final z std %.2f p99/3 %.2f'%(g.z3.std(),np.percentile(np.abs(g.z3),99)/3))
    out[cls]=dict(r=centers,k=[float(x) for x in ks],cedge=cedge,cf=[float(x) for x in cfarr])
    if cls=='ball': out['lqFac']=float(lq)
# bearing inflation
kb={}
for cls,cmin in [('ball',.6),('robot',.2)]:
    g=d[(d.cls==cls)&(d.conf>=cmin)]
    be=np.degrees(np.angle(np.exp(1j*(g.estB-g.trueB))))
    sd=np.degrees(np.hypot(g.bsd,np.radians(0.5)))
    zb=be/sd
    kb[cls]=float(max(1.0,np.std(zb),np.percentile(np.abs(zb),99)/3))
    print(cls,'bearing z std',zb.std(),'p99/3',np.percentile(np.abs(zb),99)/3)
out['kB']=kb
json.dump(out,open('calib/k.json','w'),indent=1)
