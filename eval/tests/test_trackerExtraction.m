function test_trackerExtraction()
%TEST_TRACKEREXTRACTION (1) extracted functions are verbatim copies of the
%   local functions in run_5v5_decentralised.m; (2) 30 s synthetic smoke test
%   makePackets -> runSharedTracker -> trackEstimates, ball error sane.
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(genpath(root));

% (1) verbatim copy check
scr = fileread(fullfile(root, 'run_5v5_decentralised.m'));
scr = regexprep(scr, '\r', '');
% (processPacket, gateStats, mergeDuplicates, fuseDetection, makeMeasurement
% and spawnTrack have since gained OPTIONAL knobs whose defaults reproduce the
% script's behaviour, so they are no longer textual copies.)
names = {'expectedVisible', 'makeEvent'};
for i = 1:numel(names)
    src = regexprep(fileread(fullfile(root, 'eval', 'tracker', [names{i} '.m'])), '\r', '');
    assert(contains(scr, strtrim(src)), '%s differs from the original script', names{i});
end
fprintf('verbatim-copy check passed (%d functions)\n', numel(names));

% (2) smoke test
rng(1);
dt = 0.05; T = 30; t = (0:dt:T).'; Nt = numel(t);
gt.t = t; gt.team = [1 1 1 2 2 2];
% ball: piecewise kicks with friction decay
kappa = 0.2; bx = [0 0]; bv = [1 0.5]; ball = zeros(Nt, 2);
kicks = {[8 -4 2.5], [14 3 -3], [21 -5 1]};
for k = 1:Nt
    ball(k, :) = bx;
    for j = 1:numel(kicks)
        if abs(t(k) - kicks{j}(1)) < dt/2, bv = bv + kicks{j}(2:3); end
    end
    bx = bx + bv*(1 - exp(-kappa*dt))/kappa; bv = bv*exp(-kappa*dt);
    bx = max(min(bx, [6.5 4]), [-6.5 -4]);
end
gt.ball = ball;
c0 = [-3 -2; -3 2; -5 0; 3 -2; 4 2; 2 0];
rad = [1.5 2 1 2 1.5 2]; om = [0.2 -0.15 0.25 0.2 -0.2 0.15];
gt.pos = zeros(Nt, 2, 6); gt.theta = zeros(Nt, 6);
for j = 1:6
    a = om(j)*t + j;
    gt.pos(:, :, j) = c0(j, :) + rad(j)*[cos(a) sin(a)];
    gt.theta(:, j) = a + sign(om(j))*pi/2;
end
params = struct();
tic; packets = makePackets(gt, params); tPk = toc;
fprintf('%d packets, %d detections (%.1fs)\n', numel(packets), ...
    sum(arrayfun(@(q) numel(q.detections), packets)), tPk);
delivered = rand(3, numel(packets)) >= 0.2;
tic; copies = runSharedTracker(packets, delivered, params); tTr = toc;
fprintf('runSharedTracker: %.1f s for %.0f s sim -> %.1f s wall per sim-minute\n', tTr, T, tTr/T*60);

tEval = 3:0.5:T;
for r = 1:numel(copies)
    errB = nan(size(tEval)); errO = nan(numel(tEval), 3); inReg = nan(size(tEval));
    for i = 1:numel(tEval)
        k = find(gt.t >= tEval(i), 1);
        eb = trackEstimates(copies(r), tEval(i), 'ball');
        if ~isempty(eb)
            d = arrayfun(@(e) norm(e.mean - gt.ball(k, :).'), eb);
            [errB(i), j] = min(d);
            e = eb(j).mean - gt.ball(k, :).';
            inReg(i) = e.' * (eb(j).cov \ e) <= chi2inv(0.95, 2);
        end
        er = trackEstimates(copies(r), tEval(i), 'robot');
        for o = 1:3
            if ~isempty(er)
                errO(i, o) = min(arrayfun(@(e) norm(e.mean - squeeze(gt.pos(k, :, 3+o)).'), er));
            end
        end
    end
    fprintf('copy %d: ball avail %.2f, median err %.2f m, frac<1m %.2f, in95 %.2f | opp median err %.2f m\n', ...
        r, mean(~isnan(errB)), median(errB, 'omitnan'), mean(errB < 1), mean(inReg, 'omitnan'), median(errO(:), 'omitnan'));
    assert(mean(errB < 1) > 0.5, 'copy %d: ball error too large', r);
end
fprintf('test_trackerExtraction PASSED\n');
end
