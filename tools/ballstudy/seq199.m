% Event sequence around t~190-199 (real01): ball detections and ball tracks, per copy.
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); cd(root); addpath(genpath(root));
S = load(getenv('JOINT_MAT')); joint = S.joint; packets = S.packets; delivered = S.delivered;
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
t0 = str2double(getenv('T0')); t1 = str2double(getenv('T1'));
fprintf('--- ball detections t in [%g,%g]\n', t0, t1);
for p = 1:numel(packets)
    pk = packets(p); if pk.time < t0 || pk.time > t1, continue, end
    for d = pk.detections
        if ~strcmp(d.Class, 'ball'), continue, end
        i = find(gt.t >= pk.time, 1); tb = gt.ball(i, :);
        u = d.y(1:2); u = u(:)/norm(u); zp = pk.ObserverPosition(:) + d.y(3)*u;
        rs = nan; cf = nan; if isfield(d, 'LastDetection') && isstruct(d.LastDetection), rs = d.LastDetection.RangeStd; cf = d.LastDetection.Conf; end
        fprintf('t=%.2f robot%d deliveredTo=%s range=%.2f rangeStd=%.2f conf=%.2f meas=(%.2f,%.2f) truth=(%.2f,%.2f) err=%.2f\n', pk.time, pk.RobotID, mat2str(double(delivered(:,p)')), d.y(3), rs, cf, zp, tb, norm(zp(:)-tb(:)));
    end
end
for r = 1:numel(joint)
    tr = joint(r).tracks;
    for k = find(strcmp({tr.Class}, 'ball'))
        T = tr(k); qt = [T.queue.time];
        if T.SpawnTime > t1 || (~T.alive && T.deletedAt < t0) , continue, end
        m = T.queue(1).system.density.mean(); 
        fprintf('copy %d %s spawn=%.2f del=%.2f(%s) nfused=%d firstfuse t=%.2f lastfuse t=%.2f\n', r, T.Name, T.SpawnTime, T.deletedAt, T.DeleteReason, numel(qt), qt(1), qt(end));
        for q = T.queue(arrayfun(@(x) x.time >= t0 && x.time <= t1, T.queue))
            mm = q.system.density.mean(); C = q.system.density.cov(); i = find(gt.t >= q.time, 1);
            fprintf('    t=%.2f robot%d post-mean=(%.2f,%.2f) std=%.2f truth=(%.2f,%.2f) err=%.2f\n', q.time, q.RobotID, mm(1), mm(2), sqrt(max(eig(C(1:2,1:2)))), gt.ball(i,:), norm(mm(1:2)'-gt.ball(i,:)));
        end
    end
end
