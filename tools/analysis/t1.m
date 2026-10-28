% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
gt = loadGroundTruth('eval/gt/real01.csv', 0.05);
fprintf('N=%d dt=%.3f T=%.1f\n', numel(gt.t), median(diff(gt.t)), gt.t(end));
ev = teleports(gt, 8);
for e = ev, fprintf('%.2f->%.2f (%.2f,%.2f)->(%.2f,%.2f) d=%.2f out=%d\n', e.t0,e.t1,e.from,e.to,e.dist,e.outBefore); end
ev2 = teleports(gt, 15); fprintf('%d at 15m/s\n', numel(ev2));
b=gt.ball; fprintf('max |x| %.2f max|y| %.2f\n', max(abs(b(:,1))), max(abs(b(:,2))));
% out of field excursions
out = abs(b(:,1))>7 | abs(b(:,2))>4.5; d=diff([0;out;0]); s=find(d==1); e=find(d==-1)-1;
for i=1:numel(s), fprintf('out %.2f-%.2f at (%.2f,%.2f)\n', gt.t(s(i)), gt.t(e(i)), b(s(i),:)); end
v=hypot(diff(b(:,1)),diff(b(:,2)))./diff(gt.t); fprintf('speed pct 50/99/99.9: %.2f %.2f %.2f\n', prctile(v,[50 99 99.9]));
