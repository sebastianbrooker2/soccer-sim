import sys, numpy as np, pandas as pd
f = sys.argv[1]
d = pd.read_csv(f)
d['tp'] = ~d.label.eq('fp')
d['rerr'] = d.estR - d.trueR
d['berr'] = np.degrees(np.angle(np.exp(1j*(d.estB - d.trueB))))
d['rel'] = d.rerr/d.trueR
bands = [0,2,4,6,8,10,100]
d['band'] = pd.cut(d.trueR, bands, right=False)
def st(g, col):
    x = g[col].dropna()
    return pd.Series(dict(n=len(x), bias=x.mean(), std=x.std(), mad=1.4826*np.median(np.abs(x-np.median(x))), p99=np.percentile(np.abs(x),99) if len(x) else np.nan))
for cls in ['ball','robot']:
    for m in ['size','ground','fuse']:
        g = d[(d.cls==cls)&(d.method==m)&d.tp]
        print(f'--- {cls} {m}: range error (m) / relative / bearing(deg) by TRUE distance')
        r = g.groupby('band', observed=True).apply(lambda x: pd.concat([st(x,'rerr').add_prefix('R_'), st(x,'rel').add_prefix('rel_')[['rel_bias','rel_std']], st(x,'berr')[['bias','std']].add_prefix('B_')]))
        print(r.round(3).to_string())
    g = d[(d.cls==cls)&(d.method=='fuse')]
    print(f'--- {cls} precision by (true or est) distance, fuse (est range band) ')
    g = g.assign(eb=pd.cut(g.estR, bands, right=False))
    print(g.groupby('eb', observed=True).agg(n=('tp','size'), prec=('tp','mean')).round(3).to_string())
    for lo,hi in [(0,.5),(.5,.6),(.6,.7),(.7,.8),(.8,1.01)]:
        pass
    g2 = g.assign(cb=pd.cut(g.conf,[0,.3,.4,.5,.6,.7,.8,1.01]))
    print(g2.groupby('cb', observed=True).agg(n=('tp','size'), prec=('tp','mean')).round(3).to_string())
    g3 = g.assign(wb=pd.cut(g.w,[0,4,5,6,8,10,15,30,400]))
    print(g3.groupby('wb', observed=True).agg(n=('tp','size'), prec=('tp','mean')).round(3).to_string())
