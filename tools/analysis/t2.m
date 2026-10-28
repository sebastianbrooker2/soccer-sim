% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
for f=[1 3 6]
gt = loadGroundTruth(sprintf('eval/gt/real%02d.csv',f), 0.05); b=gt.ball; t=gt.t;
v=[0;hypot(diff(b(:,1)),diff(b(:,2)))./diff(t)];
a=[0;diff(v)]/0.05;
fprintf('== real%02d\n',f);
for i=find(v>8)'; fprintf('  v>8 t=%.2f (%.2f,%.2f)->(%.2f,%.2f) v=%.1f\n',t(i),b(i-1,:),b(i,:),v(i)); end
% impulsive onset: v(i-1)<0.15 & v(i)>1
k=find(v(1:end-1)<0.15 & v(2:end)>1.0)+1;
for i=k'
  dr=min(hypot(squeeze(gt.pos(i,1,:))-b(i,1),squeeze(gt.pos(i,2,:))-b(i,2)));
  fprintf('  onset t=%.2f at (%.2f,%.2f) v=%.2f nearest robot %.2f\n',t(i),b(i,:),v(i),dr);
end
end
