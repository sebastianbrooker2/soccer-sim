% Sanity check of ground-plane axis/sign conventions for YOLO measurements vs GT on real01. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
params = evalParams(); params.tracker.spawnAnalyticBall = true; gt = loadGroundTruth('eval/gt/real01.csv', 0.05); rng(1);
packets = makePacketsYolo(gt, params, 'recordings/real01/yolo');
rid = arrayfun(@(p) p.RobotID, packets);
numObs = 3; out = [];
for r = 1:3
    dm = false(numObs, numel(packets)); dm(r, rid==r) = true;
    sr = runSharedTracker(packets, dm, params); c = sr(r);
    for i = 1:numel(c.tracks)
        T = c.tracks(i); if ~strcmp(T.Class,'ball') || isempty(T.queue), continue; end
        % first and second fusion
        for q = 1:min(2, numel(T.queue))
            s = T.queue(q).system; m = s.density.mean(); C = s.density.cov(); Pp = C(1:2,1:2);
            t = T.queue(q).time;
            kk = min(max(round((t-gt.t(1))/(gt.t(2)-gt.t(1)))+1,1),numel(gt.t));
            % observer believed position: from the packet nearest in time for this robot
            ip = find(rid==r & abs([packets.time]-t)<1e-6,1); if isempty(ip), [~,ip]=min(abs([packets.time]-t)+1e3*(rid~=r)); end
            o = packets(ip).ObserverPosition(:);
            d = m(1:2)-o; ray = d/norm(d); [V,D] = eig(Pp); [lam,ix] = sort(diag(D),'descend'); v1 = V(:,ix(1));
            ang = acosd(min(1,abs(v1.'*ray)));
            out(end+1,:) = [r i q t norm(d) sqrt(lam(1)) sqrt(lam(2)) ang]; %#ok<AGROW>
        end
    end
end
fprintf('robot track fusion t range_to_ball sd_major sd_minor angle(major axis vs line of sight, deg)\n');
fprintf('%d %3d %d %7.1f %5.1f %6.2f %6.2f %5.1f\n', out.');
fprintf('median angle (fusion 1): %.1f deg; (fusion 2): %.1f deg\n', median(out(out(:,3)==1,8)), median(out(out(:,3)==2,8)));
