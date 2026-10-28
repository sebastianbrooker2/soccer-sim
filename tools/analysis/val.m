% Validation of YOLO/detection packets against det01 GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
gt = loadGroundTruth('eval/gt/det01.csv'); fr = readtable('recordings/det01/det_frames.csv'); dt = readtable('recordings/det01/det.csv');
db = dt(strcmp(dt.class,'Ball'),:);
cam = k1SimCamera(0.55); opt = struct('zfar',inf,'znear',0.01,'minRobotPx',0);
fprintf('gt t %.2f..%.2f, det t %.2f..%.2f\n', gt.t(1), gt.t(end), min(fr.t), max(fr.t));
res = []; 
for i=1:height(fr)
  r=fr.robot(i); t=fr.t(i); kk=round((t-gt.t(1))/(gt.t(2)-gt.t(1)))+1; if kk<1||kk>numel(gt.t), continue, end
  [cp,Rc]=cameraPoseK1(cam,[squeeze(gt.pos(kk,:,r)).';gt.z(kk,r)],gt.theta(kk,r),gt.roll(kk,r),gt.pitch(kk,r),gt.headYaw(kk,r),gt.headPitch(kk,r));
  o=cameraProject('ball',[gt.ball(kk,:).';0.107],cp,Rc,cam,opt);
  m = db.robot==r & abs(db.t-t)<1e-6;
  res(end+1,:) = [r t o.visible any(m) o.depth o.ratio o.full(1:2) o.full(3) sum(m) NaN NaN]; %#ok<AGROW>
  if any(m), res(end,11:12)=[db.u_px(find(m,1)) db.w_px(find(m,1))]; else, res(end,11:12)=NaN; end
end
save('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/val.mat','res');
v=res(:,3)==1; s=res(:,4)==1;
fprintf('frames %d | sim ball %d, model %d | TP %d FP %d FN %d TN %d\n', size(res,1), sum(s),sum(v),sum(v&s),sum(v&~s),sum(~v&s),sum(~v&~s));
fprintf('precision %.4f recall %.4f agreement %.4f\n', sum(v&s)/sum(v), sum(v&s)/sum(s), mean(v==s));
d=res(s,5); fprintf('sim-detected depth range %.2f..%.2f m; ratio min %.3f\n', min(d), max(d), min(res(s,6)));
e=res(s&v,[7 11]); fprintf('u model vs sim: median abs %.2f px; w median abs diff %.2f px\n', median(abs(e(:,1)-e(:,2))), median(abs(res(s&v,9)-res(s&v,12))));
bad=res(v&~s|~v&s,:); disp(bad(1:min(15,end),[1 2 3 4 5 6 7 8]));
