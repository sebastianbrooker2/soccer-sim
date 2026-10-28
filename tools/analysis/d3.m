% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
load('d1.mat','gt','pk','params','cb','cs');
for r=1:3
 tr=cs(r).tracks; cl={tr.Class}; lab={tr.TrueLabel};
 fprintf('shared r%d: %d tracks (%d ball) alive %d\n',r,numel(tr),sum(strcmp(cl,'ball')),sum([tr.alive]));
 [u,~,j]=unique(lab); disp([u;num2cell(accumarray(j(:),1))']);
end
% NEES using label-based truth for shared tracks, per eval time: live tracks of label opp o; min-dist track
tE=(1:0.5:299)'; nees=[]; nn=0; 
for r=1:3
 for te=tE'
  E=trackEstimates(cs(r),te,'robot');
  for o=4:6
   p=interp1(gt.t,squeeze(gt.pos(:,:,o)),te);
   for j=1:numel(E)
     if any(strcmp(E(j).name,{cs(r).tracks.Name})) 
     end
   end
  end
 end
end
% simpler: count live robot tracks per eval time (expected 5 incl teammates 2 other + 3 opp)
cnt=[]; for r=1:3, for te=tE', cnt(end+1)=numel(trackEstimates(cs(r),te,'robot')); end, end
fprintf('live robot tracks: mean %.2f, hist: %s\n',mean(cnt),mat2str(histcounts(cnt,-0.5:1:10.5)));
cnt=[]; for r=1:3, for te=tE', cnt(end+1)=numel(trackEstimates(cs(r),te,'ball')); end, end
fprintf('live ball tracks: mean %.2f hist %s\n',mean(cnt),mat2str(histcounts(cnt,-0.5:1:5.5)));
% oracle-nearest NEES distribution for opponents, and 'wrong-label' check using ev name->TrueLabel
bad=0;tot=0; nees=[]; ee=[];
for r=1:3, for te=tE'
  E=trackEstimates(cs(r),te,'robot'); if isempty(E), continue, end
  for o=4:6
    p=interp1(gt.t,squeeze(gt.pos(:,:,o)),te)';
    M=[E.mean]; d=vecnorm(M-p); [e,j]=min(d); 
    nm=E(j).name; tk=cs(r).tracks(strcmp({cs(r).tracks.Name},nm)); 
    tot=tot+1; if ~strcmp(tk.TrueLabel,sprintf('opp%d',o-3)), bad=bad+1; end
    ev=M(:,j)-p; nees(end+1)=ev'*(E(j).cov\ev); ee(end+1)=e;
  end
end, end
fprintf('oracle match label mismatch frac %.3f (n=%d)\n',bad/tot,tot);
fprintf('NEES(2dof) mean %.2f median %.2f (nominal 2, 1.39); frac<=5.99 %.3f; err p50/90/99 %.2f %.2f %.2f\n',mean(nees),median(nees),mean(nees<=5.99),prctile(ee,[50 90 99]));
fprintf('NEES pct 90 %.1f 99 %.1f\n',prctile(nees,[90 99]));
