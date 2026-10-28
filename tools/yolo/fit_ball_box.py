import numpy as np, pandas as pd
d = pd.read_csv('calib/before.csv'); d = d[(d.method=='fuse')&(d.cls=='ball')&(d.label=='ball')]
print(len(d))
for c in [(0.1,2),(0.6,2)]:
    pass
d['wb']=pd.cut(d.tw,[0,4,5,6,8,10,14,20,30,60])
d['e']=d.w-d.tw; d['eh']=d.h-d.tw
print(d.groupby('wb',observed=True).agg(n=('e','size'),tw=('tw','mean'),w=('w','mean'),w_bias=('e','mean'),w_std=('e','std'),h_bias=('eh','mean'),h_std=('eh','std')).round(2))
print(d.groupby(pd.cut(d.conf,[0,.4,.6,.8,1.01]),observed=True).agg(n=('e','size'),w_bias=('e','mean'),w_std=('e','std'),h_bias=('eh','mean')).round(2))
for conf in (0.1,0.6):
    g=d[d.conf>=conf]
    A=np.c_[g.w,np.ones(len(g))]; co=np.linalg.lstsq(A,g.tw,rcond=None)[0]
    A2=np.c_[(g.w+g.h)/2,np.ones(len(g))]; co2=np.linalg.lstsq(A2,g.tw,rcond=None)[0]
    print(conf,'tw=a*w+b',co,'resid std',np.std(g.tw-A@co),' with mean(w,h)',co2,np.std(g.tw-A2@co2))
# vertical centre residual v - tv
d['ev']=d.v-d.tv; d['eu']=d.u-d.tu
print(d.groupby('wb',observed=True).agg(ev_bias=('ev','mean'),ev_std=('ev','std'),eu_bias=('eu','mean'),eu_std=('eu','std')).round(2))
