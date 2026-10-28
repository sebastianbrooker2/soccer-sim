function E = loadEvents(csvFile)
%LOADEVENTS Read events_*.csv (t,type,id,from_x,from_y,to_x,to_y,kind) written by exportEvents.
%   For overlays: E(E.type=="ball",:), E(E.type=="robot",:). Use
%   robotResetMatrix(E) / ballResetTimes(E,kinds) to feed params.tracker.
if nargin < 1, csvFile = fullfile(fileparts(mfilename('fullpath')), 'events_real01.csv'); end
E = readtable(csvFile, 'TextType', 'string');
end
