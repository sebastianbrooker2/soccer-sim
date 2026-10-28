function x = trueStateAt(label, t, truth)
%TRUESTATEAT Ground-truth [position; velocity] for LABEL at time T.
%   Debug/plotting only: the tracker never reads it. Three modes:
%     * TRUTH empty            -> zeros(4,1) (no ground truth available)
%     * TRUTH.gt (eval struct) -> interpolated from gt (labels 'ball',
%       'team<j>' = gt id j, 'opp<o>' = gt id 3+o); velocity by finite diff.
%     * TRUTH.dtSim (original run_5v5_decentralised.m grids) -> original code.

if isempty(truth)
    x = zeros(4, 1);
    return
end

if isfield(truth, 'gt')
    gt = truth.gt;
    if strcmp(label, 'ball')
        P = gt.ball;
    elseif startsWith(label, 'opp')
        P = squeeze(gt.pos(:, :, 3 + sscanf(label, 'opp%d'))); 
    elseif startsWith(label, 'team')
        P = squeeze(gt.pos(:, :, sscanf(label, 'team%d')));
    else
        error('Unknown TrueLabel ''%s''', label);
    end
    P = reshape(P, [], 2);
    Nt = numel(gt.t);
    kk = min(max(round((t - gt.t(1)) / (gt.t(2) - gt.t(1))) + 1, 1), Nt);
    k2 = min(kk + 1, Nt); k1 = max(kk - 1, 1);
    v = (P(k2, :) - P(k1, :)).' / max(gt.t(k2) - gt.t(k1), eps);
    x = [P(kk, :).'; v];
    return
end

kk = min(max(round(t/truth.dtSim) + 1, 1), truth.Nt);

if strcmp(label, 'ball')
    x = [truth.ballPos(:, kk); truth.ballVel(:, kk)];
elseif startsWith(label, 'opp')
    o = sscanf(label, 'opp%d');
    x = [truth.oppPos{o}(:, kk); truth.oppVel{o}(:, kk)];
elseif startsWith(label, 'team')
    j = sscanf(label, 'team%d');
    x = [truth.robots(j).Position(:); 0; 0];
else
    error('Unknown TrueLabel ''%s''', label);
end
end
