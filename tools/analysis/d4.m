% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
load('d1.mat','gt','pk','params','cb','cs');
L=cs(1).log; ty={L.type}; fprintf('events: '); [u,~,j]=unique(ty); disp([u;num2cell(accumarray(j(:),1))']);
sp=L(strcmp(ty,'spawn')); fprintf('spawn notes: '); [u,~,j]=unique({sp.note}); disp([u;num2cell(accumarray(j(:),1))']);
as=L(strcmp(ty,'assoc')); fprintf('assoc correct label frac %.3f of %d\n',mean([as.flag]),numel(as));
mg=L(strcmp(ty,'merge')); fprintf('merges %d correct %.3f\n',numel(mg),mean([mg.flag]));
d=L(strcmp(ty,'delete')); fprintf('deletes %d median life %.2f\n',numel(d),median([d.life]));
% duplicate spawns: d2S distribution & spawn position std/err
dup=sp(strcmp({sp.note},'duplicate')); fprintf('dup spawn d2S pct 10/50/90: %.1f %.1f %.1f ; gate %.1f\n',prctile([dup.d2S],[10 50 90]),chi2inv(2*normcdf(3)-1,3));
fprintf('spawn pos std after first fusion median %.2f, err median %.2f\n',median([sp.life]),median([sp.aux]));
% spawn by label for duplicates
[u,~,j]=unique({dup.trackLabel}); disp([u;num2cell(accumarray(j(:),1))']);
% are those spawns rejected because of gate: check track lifetimes for opp2
tr=cs(1).tracks; o2=tr(strcmp({tr.TrueLabel},'opp2')); 
life=arrayfun(@(t) (isnan(t.deletedAt)*300+~isnan(t.deletedAt)*t.deletedAt)-t.SpawnTime,o2); fprintf('opp2 tracks life median %.1f, n fused median %d\n',median(life),median(arrayfun(@(t) numel(t.queue),o2)));
% where is opp2 relative to others? min dist to other robots
dd=[]; for k=1:20:size(gt.pos,1), P=squeeze(gt.pos(k,:,:)); dd(end+1,:)=vecnorm(P-P(:,5))'; end
fprintf('opp2 dist to others median: %s\n',mat2str(median(dd),2));
