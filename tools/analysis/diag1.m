% Ad-hoc debugging of tracker behaviour (spawn/merge/gating) on real01 / synthetic GT. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
params = evalParams();
gt = loadGroundTruth('/home/seb/soccer-sim/eval/gt/real01.csv', 0.05);
tmax=400; keep = gt.t-gt.t(1)<=tmax; gt.t=gt.t(keep); gt.ball=gt.ball(keep,:); gt.pos=gt.pos(keep,:,:); gt.theta=gt.theta(keep,:);
for f={'z','roll','pitch','headYaw','headPitch'}, if isfield(gt,f{1}), gt.(f{1})=gt.(f{1})(keep,:); end, end
rng(1);
packets = makePacketsYolo(gt, params, '/home/seb/soccer-sim/recordings/real01/yolo');
% pose error normalised
pe=[]; 
for p=1:numel(packets)
  k=min(max(round((packets(p).time-gt.t(1))/0.05)+1,1),numel(gt.t));
  r=packets(p).RobotID; pe(end+1,:)=[packets(p).RobotID packets(p).time (packets(p).ObserverPosition(:)'-squeeze(gt.pos(k,:,r)))]; 
end
fprintf('pose err std x %.3f y %.3f (nominal 0.1), lag1 corr over frames %.3f\n', std(pe(:,3)), std(pe(:,4)), corr(pe(1:end-1,3),pe(2:end,3)));
fprintf('pose err 5s-apart corr: %.3f\n', corr(pe(1:end-75,3),pe(76:end,3)));
% measurement error: range along / bearing across, normalised
rows=[];
for p=1:numel(packets)
  pk=packets(p);
  for d=1:numel(pk.detections)
    dt=pk.detections(d); if any(isnan(dt.TrueDelta)), continue, end
    y=dt.y; u=y(1:2); r=y(3); 
    td=dt.TrueDelta; rt=norm(td); ut=td/rt;
    eR=r-rt; ang=atan2(u(2),u(1))-atan2(ut(2),ut(1)); ang=mod(ang+pi,2*pi)-pi; eB=r*ang;
    sR=dt.LastDetection.RangeStd; sB=r*hypot(dt.LastDetection.BearingStd, deg2rad(0.5)) ;
    lq=dt.LastDetection.LowQuality; cf=dt.LastDetection.Conf;
    rows(end+1,:)=[strcmp(dt.Class,'ball') r eR/sR eB/(r*pk.DetectionNoiseStd) sR cf lq rt]; %#ok
  end
end
save diag_rows.mat rows pe
for c=[1 0]
  m=rows(:,1)==c & abs(rows(:,3))<50;
  z=rows(m,:); fprintf('class ball=%d n=%d: range z: std %.2f robust(MAD*1.48) %.2f frac|z|<1: %.2f <2: %.2f <3: %.2f ; bearing z std %.2f robust %.2f frac<2 %.2f; chi2(2d) mean %.2f med %.2f (cal 2/1.39)\n', c, sum(m), std(z(:,3)), 1.4826*median(abs(z(:,3))), mean(abs(z(:,3))<1), mean(abs(z(:,3))<2), mean(abs(z(:,3))<3), std(z(:,4)), 1.4826*median(abs(z(:,4))), mean(abs(z(:,4))<2), mean(z(:,3).^2+z(:,4).^2), median(z(:,3).^2+z(:,4).^2));
  % chi2 by conf band
  for e=[0 0.5 0.8;0.5 0.8 1.01]'
    mm=z(:,6)>=e(1)&z(:,6)<e(2); if sum(mm)>20, fprintf('   conf[%.1f,%.1f) n=%d  range z std %.2f (rob %.2f), chi2 mean %.2f, frac95 (chi2<5.99): %.3f\n', e(1),e(2),sum(mm), std(z(mm,3)),1.4826*median(abs(z(mm,3))), mean(z(mm,3).^2+z(mm,4).^2), mean(z(mm,3).^2+z(mm,4).^2<5.99)); end
  end
end
