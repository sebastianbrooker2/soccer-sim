function copies = runSharedTracker(packets, delivered, params)
%RUNSHAREDTRACKER Cooperative tracker: every robot runs its own copy of the
%   info-form filter on all packets it received (loop of
%   run_5v5_decentralised.m :310-346).
%   packets   : struct array sorted by time (see makePackets)
%   delivered : numRobots x numPackets logical, delivered(r,p) = robot r
%               received packet p ([] = everything delivered). The sender's
%               own packet is always delivered.
%   params    : eval params (see trackerParamsFill); optional
%               params.truth (script-style truth or struct('gt',gt)) is used
%               ONLY for debug labels/x_sim, never by the filter.
%   copies(r).tracks / .log as in the script.
if nargin < 3, params = struct(); end
tp = trackerParamsFill(params);
truth = evalParamOr(params, 'truth', []);
numPackets = numel(packets);
numTeam = max([packets.RobotID]);
if isempty(delivered)
    delivered = true(numTeam, numPackets);
else
    for p = 1:numPackets
        delivered(packets(p).RobotID, p) = true;
    end
    numTeam = size(delivered, 1);
end
copies = struct('tracks', cell(1, numTeam), 'log', cell(1, numTeam));
for r = 1:numTeam
    tracks = struct('Name', {}, 'Class', {}, 'TrueLabel', {}, 'system', {}, 'initialSystem', {}, ...
        'queue', {}, 'misses', {}, 'alive', {}, 'deletedAt', {}, 'SpawnTime', {}, 'DeleteReason', {}, 'distinct', {}, 'rej', {});
    evLog = makeEvent('x', 0, '', '', '', 0, 0, 0, false, '', 0);
    evLog(1) = [];
    for p = 1:numPackets
        if delivered(r, p)
            [tracks, evs] = processPacket(tracks, packets(p), tp, truth);
            evLog = [evLog, evs]; %#ok<AGROW>
        end
    end
    copies(r).tracks = tracks;
    copies(r).log    = evLog;
end
end
