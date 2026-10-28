import sys, numpy as np, pandas as pd
pd.set_option('display.width',250)
bands=[0,2,4,6,8,10,100]
def prep(f):
    d=pd.read_csv(f); d=d[(d.method=='fuse')]
    d['tp']=d.label!='fp'
    d['e']=d.estR-d.trueR
    d['z']=d.e/d.rstd
    d['berr']=np.degrees(np.angle(np.exp(1j*(d.estB-d.trueB))))
    ex=d.estR*np.cos(d.estB); ey=d.estR*np.sin(d.estB); tx=d.trueR*np.cos(d.trueB); ty=d.trueR*np.sin(d.trueB)
    d['perr']=np.hypot(ex-tx,ey-ty)
    d['band']=pd.cut(d.trueR,bands,right=False)
    return d
def tab(g):
    r=[]
    for b,x in g.groupby('band',observed=True):
        r.append(dict(band=str(b),n=len(x),rbias=x.e.mean(),rstd=x.e.std(),sig_med=x.rstd.median(),z_std=x.z.std(),in3=(np.abs(x.z)<=3).mean()*100,
            perr_med=x.perr.median(),perr_rms=np.sqrt((x.perr**2).mean())))
    x=g; r.append(dict(band='all',n=len(x),rbias=x.e.mean(),rstd=x.e.std(),sig_med=x.rstd.median(),z_std=x.z.std(),in3=(np.abs(x.z)<=3).mean()*100,perr_med=x.perr.median(),perr_rms=np.sqrt((x.perr**2).mean())))
    return pd.DataFrame(r).round(2).set_index('band')
B=prep(sys.argv[1]); A=prep(sys.argv[2])
for cls,c in [('ball',.6),('robot',.2)]:
    for nm,D in [('BEFORE',B),('AFTER',A)]:
        g=D[(D.cls==cls)&D.tp&(D.conf>=c)]
        print(f'== {cls} {nm} (TP, conf>={c}, fused)'); print(tab(g).to_string())
    # bearing
    for nm,D in [('BEFORE',B),('AFTER',A)]:
        g=D[(D.cls==cls)&D.tp&(D.conf>=c)]
        print(nm,'bearing err std deg %.2f, p99 %.2f'%(g.berr.std(),np.percentile(np.abs(g.berr),99)))
# half-split sanity (in-sample check by time halves)
g=A[(A.cls=='ball')&A.tp&(A.conf>=.6)]; m=g.t.median()
for nm,x in [('t<med',g[g.t<m]),('t>=med',g[g.t>=m])]: print('ball after',nm,len(x),'z std %.2f in3 %.1f%%'%(x.z.std(),(np.abs(x.z)<=3).mean()*100))
g=A[(A.cls=='robot')&A.tp&(A.conf>=.2)]; m=g.t.median()
for nm,x in [('t<med',g[g.t<m]),('t>=med',g[g.t>=m])]: print('robot after',nm,len(x),'z std %.2f in3 %.1f%%'%(x.z.std(),(np.abs(x.z)<=3).mean()*100))
# LQ share
g=A[(A.cls=='ball')&(A.conf>=.6)]
print('ball conf>=.6 LowQuality: share %.2f precision LQ %.2f precision non-LQ %.2f'%(g.lowq.mean(),g[g.lowq==1].tp.mean(),g[g.lowq==0].tp.mean()))
print(g.groupby(pd.cut(g.estR,[0,2,4,6,8,10,100]),observed=True).apply(lambda x: pd.Series(dict(n=len(x),prec=x.tp.mean(),lq_share=x.lowq.mean(),prec_nonLQ=x[x.lowq==0].tp.mean(),prec_LQ=x[x.lowq==1].tp.mean()))).round(2))
