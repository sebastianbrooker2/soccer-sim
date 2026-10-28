% Raw-measurement (no tracking) metrics on real01, same packets/grid/matching as tools/compare3.
% Writes results.csv here. Run: matlab -batch "run('tools/rawmeas/run_rawmeas.m')"
here = fileparts(mfilename('fullpath')); root = fileparts(fileparts(here));
cd(root); addpath(genpath(root));
params = evalParams();
params.baseline.mode = 'faithful';
params.evalDt = 0.2; params.maxAreaForScore = 3;
gt = loadGroundTruth(fullfile(root,'eval','gt','real01.csv'), 0.05);
rng(1);
packets = makePacketsYolo(gt, params, fullfile(root,'recordings','real01','yolo'));
copies = runRawMeasurement(packets, params);
fn = @(c,t,cl) rawEstimates(c,t,cl,params);
Tt = evaluateMatch(gt, copies, fn, params, 'raw');
nTrue = struct('ball', 1, 'opponent', 3); rows = {};
% false tracks per (robot,t) sample = live estimates NOT matched one-to-one to a target of that class
% (opponents: evaluateMatch's falseTracks counts 5 robot targets incl. teammates, so recompute)
[g, ~] = findgroups(Tt.robot, Tt.t, Tt.class);

sh = splitapply(@sum, double(Tt.hasTrack), g); Tt.falseTracks = Tt.nTracks - sh(g);
for cl = {'ball','opponent'}
    for sc = 0:numel(copies)
        S = Tt(Tt.class == cl{1}, :); if sc > 0, S = S(S.robot == sc, :); end
        h = S.hasTrack; ne = S.nees(h & isfinite(S.nees)); w = S.withinR;
        rows(end+1,:) = {'Raw measurement', cl{1}, iff(sc==0,'all',sprintf('robot%d',sc)), height(S), mean(h), ...
            mean(S.inRegion95), mean(S.inRegion95(h)), mean(S.inRegion95Capped), mean(S.inRegion95Capped(h)), ...
            mean(w(:,2)), mean(w(h,2)), sqrt(mean(S.err(h).^2)), median(S.err(h)), median(S.area,'omitnan'), ...
            mean(ne), median(ne), mean(S.nTracks), mean(S.nTracks) - nTrue.(cl{1}), mean(S.falseTracks)}; %#ok<AGROW>
    end
end
vn = {'tracker','class','scope','n','availability','inRegion95','inRegion95_cond','inRegion95_capped','inRegion95_capped_cond', ...
      'within05','within05_cond','rmse','median_err','median_area95','mean_nees_cond','median_nees_cond','mean_live_tracks','live_minus_true','false_tracks_per_sample'};
R = cell2table(rows, 'VariableNames', vn);
writetable(R, fullfile(here,'results.csv')); disp(R);
function v = iff(c,a,b), if c, v=a; else, v=b; end, end
