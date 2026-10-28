function writeGroundTruthCsv(gt, csvFile)
%WRITEGROUNDTRUTHCSV Write gt struct as long-format CSV t,id,team,x,y,theta.
Nt = numel(gt.t);
fid = fopen(csvFile,'w'); c = onCleanup(@() fclose(fid));
fprintf(fid,'t,id,team,x,y,theta\n');
for i = 1:Nt
    fprintf(fid,'%.4f,0,0,%.5f,%.5f,0\n', gt.t(i), gt.ball(i,1), gt.ball(i,2));
    for k = 1:6
        fprintf(fid,'%.4f,%d,%d,%.5f,%.5f,%.5f\n', gt.t(i), k, gt.team(k), ...
            gt.pos(i,1,k), gt.pos(i,2,k), gt.theta(i,k));
    end
end
end
