% Availability gap analysis of the final tracker on real01. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
gt = loadGroundTruth('/home/seb/soccer-sim/eval/gt/real01.csv', 0.05);
dt=0.05; N=numel(gt.t);
taus=[0.5 1 2 3 5];
% velocity by central diff over 0.2 s
vel=@(P) [nan(2,2); (P(5:end,:)-P(1:end-4,:))/0.2; nan(2,2)];
vb=vel(gt.ball); 
kap=0.2;
fprintf('--- ball: pred error from true state at t0 (CV+friction kappa=0.2) ---\n');
for tau=taus
  n=round(tau/dt); i0=(3:10:N-n-3)'; 
  sp=vecnorm(vb(i0,:),2,2);
  v0=vb(i0,:); pred=gt.ball(i0,:)+v0*(1-exp(-kap*tau))/kap;
  e=vecnorm(pred-gt.ball(i0+n,:),2,2);
  vend=vb(i0+n,:); dv=vecnorm(vend-v0*exp(-kap*tau),2,2);
  free=dv<0.3; mov=sp>0.3;
  qeq=@(x) sqrt(mean(x.^2))/sqrt(tau^3/3);
  fprintf('tau %.1f: ALL med %.2f rms %.2f p90 %.2f qeq %.2f | free&moving(n=%d) med %.2f rms %.2f p90 %.2f qeq %.2f | disturbed&moving(n=%d) med %.2f rms %.2f p90 %.2f | still(n=%d) rms %.2f\n', tau, median(e), sqrt(mean(e.^2)), prctile(e,90), qeq(e), sum(free&mov), median(e(free&mov)), sqrt(mean(e(free&mov).^2)), prctile(e(free&mov),90), qeq(e(free&mov)), sum(~free&mov), median(e(~free&mov)), sqrt(mean(e(~free&mov).^2)), prctile(e(~free&mov),90), sum(~mov), sqrt(mean(e(~mov).^2)));
end
% kappa from free rolling: speed decay between consecutive samples when |a| small
sp=vecnorm(vb,2,2); ok=find(sp(1:end-20)>1.0); r=[];
for i=ok(1:5:end)'
  w=i:i+20; s=sp(w); if any(isnan(s)) , continue, end
  if all(diff(s)<=0.02) && s(end)>0.3 && (s(1)-s(end))>0.05, c=polyfit((0:20)'*dt, log(s), 1); r(end+1)=-c(1); end %#ok
end
fprintf('kappa from monotone-decaying 1 s ball segments: n=%d median %.3f  IQR [%.3f %.3f]\n', numel(r), median(r), prctile(r,25), prctile(r,75));
fprintf('--- opponents CV prediction ---\n');
for tau=taus
  n=round(tau/dt); E=[]; 
  for k=4:6, P=gt.pos(:,:,k); v=vel(P); i0=(3:10:N-n-3)'; E=[E; vecnorm(P(i0,:)+v(i0,:)*tau-P(i0+n,:),2,2), vecnorm(v(i0,:),2,2)]; end %#ok
  E=E(~isnan(E(:,1)),:); e=E(:,1);
  fprintf('tau %.1f: med %.2f rms %.2f p90 %.2f qeq %.2f ; also zero-velocity pred (vel prior 0) rms %.2f\n', tau, median(e), sqrt(mean(e.^2)), prctile(e,90), sqrt(mean(e.^2))/sqrt(tau^3/3), NaN);
end
fprintf('opp speed median %.2f p90 %.2f\n', median(E(:,2)), prctile(E(:,2),90));
% track-level growth from FINAL_s1 eval table
S=load('res_FINAL_s1.mat'); T=S.T;
edges=[0 0.25 0.75 1.5 2.5 4 7];
for cl={'ball','opponent'}
  m=strcmp(T.class,cl{1}) & T.hasTrack & isfinite(T.area) & isfinite(T.age);
  fprintf('--- track %s: by age since last update ---\n', cl{1});
  for b=1:numel(edges)-1
    mm=m & T.age>=edges(b) & T.age<edges(b+1);
    if sum(mm)<20, continue, end
    sd=sqrt(T.area(mm)/(pi*5.9915));
    fprintf('age [%.2f,%.2f) n=%d: median geo-sd %.2f m ; median err %.2f, rms err %.2f, p90 err %.2f ; inR95 %.3f\n', edges(b),edges(b+1),sum(mm), median(sd), median(T.err(mm)), sqrt(mean(T.err(mm).^2)), prctile(T.err(mm),90), mean(T.inRegion95(mm)));
  end
end
