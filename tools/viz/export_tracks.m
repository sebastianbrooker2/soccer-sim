% Export baseline (faithful, per-robot) and joint SRIF tracks + GT for the viewer.
here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
addpath(genpath(root));
params = evalParams();
params.baseline.mode = 'faithful';
gt = loadGroundTruth(fullfile(root,'eval','gt','real01.csv'), 0.05);
rng(1);
packets = makePacketsYolo(gt, params, fullfile(root,'recordings','real01','yolo'));
numObs = numel(params.observers);
delivered = rand(numObs, numel(packets)) >= params.linkDropProb;   % keep the rng draw order (identical to before)
fprintf('%d packets\n', numel(packets));
tE = (0:0.5:floor(gt.t(end))).';
if ~exist('rawOnly', 'var'), rawOnly = false; end   % rawOnly=true: keep existing base/joint/solo rows, recompute raw only
tmp = fullfile(here,'data',sprintf('tracks_tmp_%d.csv', feature('getpid')));
fid = fopen(tmp,'w');
fprintf(fid,'src,robot,t,name,class,mx,my,cxx,cxy,cyy\n');
cps = {}; fns = {}; nms = {};
if rawOnly
    old = fopen(fullfile(here,'data','tracks.csv'),'r'); fgetl(old);
    while true
        ln = fgets(old); if ~ischar(ln), break, end
        if ~strncmp(ln,'raw,',4), fwrite(fid, ln); end
    end
    fclose(old);
else
    tic; base = runBaselineTracker(packets, params); fprintf('baseline %.1fs\n', toc);
    tic; joint = runSharedTracker(packets, delivered, params); fprintf('shared %.1fs\n', toc);
    % solo: per-robot run of the SAME shared-tracker machinery, delivered only where packet is from robot r
    solo = cell(1,numObs); rid = arrayfun(@(p) p.RobotID, packets);
    for r = 1:numObs
        dm = false(numObs, numel(packets)); dm(r, rid==r) = true;
        tic; sr = runSharedTracker(packets, dm, params); fprintf('solo r%d %.1fs\n', r, toc);
        solo{r} = sr(r);
    end
    cps = {base, joint, [solo{:}]}; fns = {@baselineEstimates, @trackEstimates, @trackEstimates}; nms = {'base','joint','solo'};
end
% raw measurement (no tracking): detections of the latest packet within params.rawmeas.freshness
cps{end+1} = runRawMeasurement(packets, params); fns{end+1} = @(c,t,cl) rawEstimates(c,t,cl,params); nms{end+1} = 'raw';
for s = 1:numel(cps)
    cp = cps{s}; fn = fns{s}; nm = nms{s};
    for r = 1:numel(cp)
        for t = tE.'
            e = fn(cp(r), t, '');
            for i = 1:numel(e)
                C = e(i).cov;
                fprintf(fid,'%s,%d,%.2f,%s,%s,%.4f,%.4f,%.6g,%.6g,%.6g\n', nm, r, t, e(i).name, e(i).class, e(i).mean(1), e(i).mean(2), C(1,1), C(1,2), C(2,2));
            end
        end
    end
end
fclose(fid);
movefile(tmp, fullfile(here,'data','tracks.csv'));   % atomic replace (same filesystem)
gtTmp = fullfile(here,'data',sprintf('gt_tmp_%d.csv', feature('getpid')));
fid = fopen(gtTmp,'w');
fprintf(fid,'t,bx,by'); for k=1:6, fprintf(fid,',x%d,y%d,th%d,hy%d',k,k,k,k); end; fprintf(fid,'\n');
for t = tE.'
    [~,i] = min(abs(gt.t-t));
    fprintf(fid,'%.2f,%.4f,%.4f',t,gt.ball(i,1),gt.ball(i,2));
    for k=1:6, fprintf(fid,',%.4f,%.4f,%.4f,%.4f',gt.pos(i,1,k),gt.pos(i,2,k),gt.theta(i,k),gt.headYaw(i,k)); end
    fprintf(fid,'\n');
end
fclose(fid);
movefile(gtTmp, fullfile(here,'data','gt.csv'));
fprintf('FOV %.4f\n', params.sensor.FOV);
