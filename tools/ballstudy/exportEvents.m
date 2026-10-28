% Write tools/ballstudy/events_real01.csv from eval/gt/real01.csv
root = fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(genpath(root));
gt = loadGroundTruth(fullfile(root, 'eval', 'gt', 'real01.csv'), 0.05);
E = gtEvents(gt);
writetable(E, fullfile(root, 'tools', 'ballstudy', 'events_real01.csv'));
fprintf('%d ball events (%d jump, %d exit, %d restart), %d robot teleports\n', sum(E.type=="ball"), ...
    sum(E.kind=="jump"), sum(E.kind=="exit"), sum(E.kind=="restart"), sum(E.type=="robot"));
disp(E);
