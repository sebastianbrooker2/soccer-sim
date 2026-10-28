function info = makeSyntheticYoloDets(gt, outDir, opts)
%MAKESYNTHETICYOLODETS Synthetic stand-in for the real YOLO detector log.
%   Writes outDir/dets.csv and outDir/dets_frames.csv (schema of
%   tools/README_yolo_detections.md and MAKEPACKETSYOLO) by projecting GT
%   through the K1 camera (true pose), then adding misses, false positives,
%   confidence variation, pixel noise. Teammates ARE detected as Robot
%   (a real detector cannot tell them apart); goalposts are emitted too.
%   OPTS: rate 15 Hz, pBallMiss .25, pRobMiss .35, fpRate (per frame) .05,
%   pixStd 2, seed (default none).
if nargin < 3, opts = struct(); end
g = @(n, d) fo(opts, n, d);
rate = g('rate', 15); pbm = g('pBallMiss', 0.25); prm = g('pRobMiss', 0.35);
fpr = g('fpRate', 0.05); px = g('pixStd', 2);
if ~isfolder(outDir), mkdir(outDir); end
obs = find(gt.team == 1, 3, 'first'); nR = size(gt.pos, 3); Nt = numel(gt.t);
t0 = gt.t(1); dtg = gt.t(2) - gt.t(1); idx = @(t) min(max(round((t - t0)/dtg) + 1, 1), Nt);
hasHead = isfield(gt, 'hasHead') && gt.hasHead && isfield(gt, 'headYaw');
hasZ = hasHead && isfield(gt, 'z');
trunkH = 0.55; if hasZ, trunkH = median(gt.z(:, obs), 'all', 'omitnan'); end
cam = k1SimCamera(trunkH);
opt = struct('ballRadius', 0.107, 'robotHeight', 0.95, 'robotWidth', 0.30, 'znear', 0.05, ...
    'zfar', 12, 'minVisible', 0.5, 'minBallPx', 0, 'minRobotPx', 8);
fd = fopen(fullfile(outDir, 'dets.csv'), 'w'); fprintf(fd, 't,robot,class,u_px,v_px,w_px,h_px,conf\n');
ff = fopen(fullfile(outDir, 'dets_frames.csv'), 'w'); fprintf(ff, 't,robot,nDets\n');
nb = 0; nrb = 0; nfp = 0; nfr = 0;
for ri = 1:numel(obs)
    r = obs(ri);
    for t = (t0 + 1/rate : 1/rate : gt.t(end))
        kk = idx(t);
        if hasHead, hy = gt.headYaw(kk, r); hp = gt.headPitch(kk, r);
        else, hy = 0.7*sin(2*pi*t/6 + 2*pi*(ri-1)/3); hp = 0.25; end
        z = trunkH; rl = 0; pc = 0;
        if hasZ, z = gt.z(kk, r); rl = gt.roll(kk, r); pc = gt.pitch(kk, r); end
        xy = squeeze(gt.pos(kk, :, r)).';
        [cp, Rc] = cameraPoseK1(cam, [xy; z], gt.theta(kk, r), rl, pc, hy, hp);
        rowsOut = {};
        for c = 0:nR
            if c == 0, kind = 'ball'; Pw = [gt.ball(kk, :).'; 0.107]; cn = 'Ball'; pm = pbm;
            else
                if c == r, continue, end
                kind = 'robot'; Pw = [squeeze(gt.pos(kk, :, c)).'; 0]; cn = 'Robot'; pm = prm;
            end
            o = cameraProject(kind, Pw, cp, Rc, cam, opt);
            if ~o.visible || rand < pm, continue, end
            b = o.bbox + px*randn(1, 4); b(3:4) = max(b(3:4), 2);
            cf = min(0.99, max(0.05, 0.85 - 0.05*o.depth + 0.1*randn));
            rowsOut{end+1} = sprintf('%.4f,%d,%s,%.2f,%.2f,%.2f,%.2f,%.3f\n', t, ri, cn, b, cf); %#ok<AGROW>
            if c == 0, nb = nb + 1; else, nrb = nrb + 1; end
        end
        if rand < fpr   % false positive anywhere in the image
            cn = {'Ball', 'Robot'}; cn = cn{1 + (rand < 0.5)};
            if strcmp(cn, 'Ball'), wh = [12 12]; else, wh = [30 70]; end
            rowsOut{end+1} = sprintf('%.4f,%d,%s,%.2f,%.2f,%.2f,%.2f,%.3f\n', t, ri, cn, ...
                rand*320, 120 + rand*110, wh(1), wh(2), 0.3 + 0.3*rand); %#ok<AGROW>
            nfp = nfp + 1;
        end
        if rand < 0.1   % goalpost (must be ignored)
            rowsOut{end+1} = sprintf('%.4f,%d,Goalpost,%.2f,%.2f,10,60,0.7\n', t, ri, rand*320, 100 + 20*rand); %#ok<AGROW>
        end
        fprintf(fd, '%s', rowsOut{:});
        fprintf(ff, '%.4f,%d,%d\n', t, ri, numel(rowsOut)); nfr = nfr + 1;
    end
end
fclose(fd); fclose(ff);
info = struct('frames', nfr, 'ball', nb, 'robot', nrb, 'falsePositives', nfp);
end
function v = fo(s, n, d)
if isfield(s, n) && ~isempty(s.(n)), v = s.(n); else, v = d; end
end
