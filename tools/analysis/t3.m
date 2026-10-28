% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
S=load('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/joint0.mat'); j=S.joint;
tic; f=fastCopies(j); toc
for tt=[100.3 250.7 500]
 a=trackEstimates(j(1),tt,'ball'); b=fastEstimates(f(1),tt,'ball'); fprintf('%d %d %g\n',numel(a),numel(b), max([0 arrayfun(@(i) norm(a(i).mean-b(i).mean),1:min(numel(a),numel(b)))]));
end
tic; for tt=100:0.2:110, fastEstimates(f(1),tt,'ball'); fastEstimates(f(1),tt,'robot'); end; toc
tic; for tt=100:0.2:104, trackEstimates(j(1),tt,'ball'); trackEstimates(j(1),tt,'robot'); end; toc
