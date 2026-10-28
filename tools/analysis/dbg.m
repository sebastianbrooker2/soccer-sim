% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
rng(1); dt=0.1; t=(0:dt:50*dt)'; gt.t=t; Nt=numel(t); gt.ball=[cos(t) sin(t)]; gt.pos=zeros(Nt,2,6);
for k=1:6, gt.pos(:,:,k)=[t*0.1+k, -k*ones(Nt,1)]; end
gt.team=[1 1 1 2 2 2]; p=struct('evalDt',dt,'radii',[0.25 0.5 1],'evalTeammates',false);
f=@(c,te,cls) mkest(gt,te,cls);
T=evaluateMatch(gt,struct('x',{1,2,3}),f,p,'a');
disp(groupsummary(T,'class','mean','hasTrack'));
function e=mkest(gt,te,cls)
if strcmp(cls,'ball'), e=struct('name','b','mean',interp1(gt.t,gt.ball,te)'+0.3*randn(2,1),'cov',0.09*eye(2),'class','ball');
else, e=struct('name',{},'mean',{},'cov',{},'class',{});
 for k=4:6, e(end+1)=struct('name','r','mean',interp1(gt.t,squeeze(gt.pos(:,:,k)),te)'+0.3*randn(2,1),'cov',0.09*eye(2),'class','robot'); end, end
end
