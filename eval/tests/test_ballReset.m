function test_ballReset()
%TEST_BALLRESET Param-gated ball/robot reset and spawn-policy options of the
%   shared tracker (processPacket): defaults off == unchanged; single-ball
%   constraint; referee-style reset events; innovation reset; spawn policy.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(root));
gt = synthTeleportGT(3, 70, 30, 3);          % teleports at t = 30, 60 (ball) and 60 (opponent)
rng(1); params = evalParams(); params.evalDt = 0.5;
packets = makePackets(gt, params);
delivered = true(numel(params.observers), numel(packets));
off = struct('ballSingle', false, 'ballLostStd', inf, 'ballReplaceStd', inf, 'ballResetN', inf, 'spawnMaxRangeStd', inf, 'spawnPending', false);
run = @(ov) runSharedTracker(packets, delivered, setfields(setfields(params.tracker, off), ov));

c0 = runSharedTracker(packets, delivered, struct());   % trackerParamsFill defaults (all options OFF)
c1 = runSharedTracker(packets, delivered, struct('ballSingle', false, 'ballLostStd', inf, 'ballResetN', inf, 'spawnMaxRangeStd', inf, 'spawnPending', false));
for r = 1:numel(c0)
    assert(numel(c0(r).tracks) == numel(c1(r).tracks), 'explicit-off differs from default');
end
fprintf('defaults-off check passed\n');

% A: single ball
cA = run(struct('ballSingle', true));
for r = 1:numel(cA)
    for t = 1:2:70
        assert(numel(trackEstimates(cA(r), t, 'ball')) <= 1, 'more than one live ball track (copy %d, t=%d)', r, t);
    end
end
fprintf('single-ball check passed\n');

% D: referee events end tracks spawned before the event
ev = gt.events; bT = ev.t(ev.type == "ball"); rM = [ev.t(ev.type == "robot") ev.from_x(ev.type == "robot") ev.from_y(ev.type == "robot")];
cD = run(struct('ballResetTimes', bT, 'robotResetEvents', rM));
nEvt = 0;
for r = 1:numel(cD)
    for k = 1:numel(cD(r).tracks)
        T = cD(r).tracks(k);
        if strcmp(T.Class, 'ball') && any(T.SpawnTime < bT & bT < 70) && T.alive
            assert(~any(T.SpawnTime < bT & bT <= 69), 'ball track survived a reset event');
        end
        nEvt = nEvt + strcmp(T.DeleteReason, 'event');
    end
end
assert(nEvt > 0, 'no track was ended by an event');
fprintf('event-reset check passed (%d tracks ended by events)\n', nEvt);

% C: innovation reset runs and produces 'reset' deletions or none, never errors
cC = run(struct('ballResetN', 2, 'ballResetD2', 40, 'ballResetVmax', 12));
fprintf('innovation-reset ran (%d ball tracks copy 1)\n', sum(strcmp({cC(1).tracks.Class}, 'ball')));

% spawn policy: a zero range-std budget forbids every ball spawn; pending mode must run
cS = run(struct('spawnMaxRangeStd', 0));
assert(~any(strcmp({cS(1).tracks.Class}, 'ball') & ~strcmp({cS(1).tracks.DeleteReason}, 'pending')), 'ball spawned despite policy');
cP = run(struct('spawnMaxRangeStd', 0.5, 'spawnPending', true, 'ballReplaceStd', 3));
fprintf('spawn-policy checks passed (pending run: %d ball tracks)\n', sum(strcmp({cP(1).tracks.Class}, 'ball')));
fprintf('test_ballReset: OK\n');
end

function s = setfields(s, ov)
f = fieldnames(ov); for i = 1:numel(f), s.(f{i}) = ov.(f{i}); end
end
