% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
S = load('/home/seb/soccer-sim/tools/compare3/compare3.mat'); W=10;
for cl={'ball','opponent'}
 J=S.T.joint(S.T.joint.class==cl{1},:); O=S.T.own(S.T.own.class==cl{1},:);
 bad = J.hasTrack & O.hasTrack & (J.err > O.err+0.5);
 badIn = O.inRegion95 & ~J.inRegion95 & J.hasTrack;           % joint overconfident/off where own contained truth
 lost = O.hasTrack&O.err<1 & ~(J.hasTrack&J.err<1);
 for nm={'bad','badIn','lost'}
  v = eval(nm{1}); wi=floor(J.t/W)*W; [u,~,ic]=unique(wi); c=accumarray(ic,v); [cs,o]=sort(c,'descend');
  fprintf('%s %s total %d: ', cl{1}, nm{1}, sum(v)); fprintf('[%d:%d] ', [u(o(1:6))'; cs(1:6)']); fprintf('\n');
  if strcmp(nm{1},'bad'), fprintf('   by robot: %s; jointminus own mean err %.2f\n', mat2str(accumarray(J.robot(v),1)'), mean(J.err(v)-O.err(v))); end
 end
 % joint vs own live ball track count when joint lost
end
