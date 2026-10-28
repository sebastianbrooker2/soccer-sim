% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params=evalParams(); rng(5); gt=synthGroundTruth(60,0.05,5); pk=makePackets(gt,params);
tp=trackerParamsFill(params);
% first few packets with robot detection
n=0;
for i=1:numel(pk)
  for d=1:numel(pk(i).detections)
    if strcmp(pk(i).detections(d).Class,'robot') && pk(i).detections(d).y(3)>2
      [tr,~]=spawnTrack(struct('TrueLabel',{},'alive',{},'Name',{}),pk(i),pk(i).detections(d),tp,struct('gt',gt),[],[]);
      lastwarn('');
      [tr2,ev,failed]=fuseDetection(tr,pk(i),pk(i).detections(d),0,0,struct('gt',gt));
      mn=tr2.system.density.mean(); P=tr2.system.density.cov();
      xT=trueStateAt(pk(i).detections(d).TrueLabel,pk(i).time,struct('gt',gt));
      [~,w]=lastwarn;
      fprintf('r=%.1f failed=%d warn=%s std=%s err=%.2f\n',pk(i).detections(d).y(3),failed,w,mat2str(sqrt(diag(P(1:2,1:2)))',3),norm(mn(1:2)-xT(1:2)));
      n=n+1; if n>=6, return, end
    end
  end
end
