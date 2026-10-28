import numpy as np, pandas as pd
d = pd.read_csv('calib/before.csv'); d = d[(d.method=='fuse')&(d.cls=='ball')]
d['tp']=d.label=='ball'
d['asp']=np.log(d.w/d.h)
d['wb']=pd.cut(d.w,[0,6,8,10,14,20,30,400])
d['cb']=pd.cut(d.conf,[0,.4,.5,.6,.7,.8,.9,1.01])
print('precision, rows=box w, cols=conf (n in parens)')
p=d.pivot_table(index='wb',columns='cb',values='tp',aggfunc='mean',observed=True).round(2)
n=d.pivot_table(index='wb',columns='cb',values='tp',aggfunc='size',observed=True)
print(p.to_string()); print(n.to_string())
d6=d[d.conf>=0.6]
print('conf>=.6: by aspect'); print(d6.groupby(pd.cut(d6.asp,[-2,-.3,-.15,-.05,.05,.15,.3,2]),observed=True).tp.agg(['size','mean']).round(2))
print('by estR (conf>=.6)'); print(d6.groupby(pd.cut(d6.estR,[0,2,4,6,8,10,100]),observed=True).tp.agg(['size','mean']).round(3))
print('by trueR for TP only counts'); 
print(d[d.tp].groupby(pd.cut(d.trueR,[0,2,4,6,8,10,100]),observed=True).size())
print(d.groupby('tp')[['w','h','conf','v']].describe().T.round(1).to_string())
