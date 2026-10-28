% dumpRows('study_real01_part1.mat', ...) -> prints rows as csv-ish
function dumpRows(varargin)
nm = {'b_avail','b_in95','b_in95cond','b_rmse','b_med','b_excess','b_multi','b_area','b_reacq','b_reacqFail','b_postIn','o_avail','o_in95','o_in95cond','o_rmse','o_false','o_reacq','o_postIn','secs'};
fid = fopen('study_merged.csv', 'w'); fprintf(fid, 'label,%s\n', strjoin(nm, ','));
for f = varargin
    S = load(f{1}); R = S.rows;
    for i = 1:size(R, 1), fprintf(fid, '%s,%s\n', R{i,1}, strjoin(arrayfun(@(x) sprintf('%.5g', x), cell2mat(R(i,2:end)), 'UniformOutput', false), ',')); end
end
fclose(fid);
end
