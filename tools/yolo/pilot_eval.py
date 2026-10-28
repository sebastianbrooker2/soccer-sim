import pickle, sys, numpy as np
sys.path.insert(0,'.')
from yolo_lib import apply_thresh, TH_DEFAULT, TH_WEBOTS
S='/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/pilot.pkl'
R=pickle.load(open(S,'rb'))
def iou(a,b):
    ax0,ax1,ay0,ay1=a[0]-a[2]/2,a[0]+a[2]/2,a[1]-a[3]/2,a[1]+a[3]/2
    bx0,bx1,by0,by1=b[0]-b[2]/2,b[0]+b[2]/2,b[1]-b[3]/2,b[1]+b[3]/2
    iw=max(0,min(ax1,bx1)-max(ax0,bx0)); ih=max(0,min(ay1,by1)-max(ay0,by0)); i=iw*ih
    return i/(a[2]*a[3]+b[2]*b[3]-i+1e-9)
def evaluate(model, rc, th, cls, name):
    nGT=nTP=nDet=nFP=0; errs=[]; confs=[]; bands={}
    nfr=0
    for r in R:
        if r['recolor']!=rc: continue
        nfr+=1
        dets=apply_thresh(r['dets'][model],th); dets=[d for d in dets if int(d[5])==cls]
        gts=[g for g in r['gt'] if g['cls']==name]
        used=set()
        # greedy by conf
        for d in sorted(dets,key=lambda d:-d[4]):
            nDet+=1
            best=-1;bi=0.2
            for j,g in enumerate(gts):
                if j in used: continue
                if g['vis']<0.3: continue
                v=iou(d,[g['cu'],g['cv'],g['cw'],g['ch']])
                if v>bi: bi=v;best=j
            if best>=0:
                used.add(best); g=gts[best]; confs.append(d[4]); errs.append((d[0]-g['cu'],d[1]-g['cv'],d[2]-g['cw'],d[3]-g['ch'],bi))
            else: nFP+=1
        for j,g in enumerate(gts):
            if g['vis']>=0.7 and g['cw']>=4 and g['ch']>=4:
                nGT+=1; b=int(min(g['depth'],9.99)//2)
                bands.setdefault(b,[0,0]); bands[b][0]+=1; bands[b][1]+= (j in used)
        nTP+=0
    tp=len(errs); e=np.array(errs) if errs else np.zeros((0,5))
    return dict(nfr=nfr,gt=nGT,det=nDet,tp=tp,fp=nFP,prec=tp/max(nDet,1),errs=e,bands=bands)
for rc in (False,True):
  for model in ['yolo_hsl_ft','yolo','old']:
    for tn,th in (('default',TH_DEFAULT),('webots',TH_WEBOTS),('low0.1',[0.1]*6)):
      for cls,name in ((0,'Ball'),(2,'Robot')):
        r=evaluate(model,rc,th,cls,name); e=r['errs']
        print(f"rc={int(rc)} {model:11s} {tn:7s} {name:5s} frames={r['nfr']} gt={r['gt']} det={r['det']} TP={r['tp']} FP={r['fp']} prec={r['prec']:.2f} recall~={r['tp']/max(r['gt'],1):.2f} cen_err_med={np.median(np.hypot(e[:,0],e[:,1])) if len(e) else 0:.1f} iou_med={np.median(e[:,4]) if len(e) else 0:.2f}")
