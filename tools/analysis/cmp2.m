% Compare tracker variants on real01 (time window / camera sensor) - ad-hoc analysis. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
gt = loadGroundTruth('eval/gt/real01.csv'); p = evalParams(); p.sensor.model='camera'; rng(2);
[pk,info] = makePacketsCamera(gt,p); disp(struct2table(info));
er=[]; ea=[]; rr=[];
for i=1:numel(pk), q=pk(i); kk=round((q.time-gt.t(1))/(gt.t(2)-gt.t(1)))+1; r=q.RobotID;
 for d=q.detections, if strcmp(d.Class,'ball'), tr=norm(gt.ball(kk,:)-squeeze(gt.pos(kk,:,r))); er(end+1)=d.y(3)-tr; rr(end+1)=tr;
   u=(gt.ball(kk,:)-squeeze(gt.pos(kk,:,r)))'/tr; ea(end+1)=acos(min(1,dot(u,d.y(1:2)/norm(d.y(1:2))))); end,end,end
fprintf('ball range err: median %.3f rms %.3f; bearing err rms %.2f deg; median range %.1f m\n', median(er), rms(er), rad2deg(rms(ea)), median(rr));
for lo=[0 3 6 9], m=rr>=lo&rr<lo+3; fprintf(' range %d-%d: n=%d rms err %.2f\n',lo,lo+3,sum(m),rms(er(m))); end
