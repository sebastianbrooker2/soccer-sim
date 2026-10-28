function animateMultiRobotSim(gt, detections, params, varargin)
%ANIMATEMULTIROBOTSIM Top-down field animation of the multi-robot sim.
%   ANIMATEMULTIROBOTSIM(GT, DETECTIONS, PARAMS) replays GT (see
%   SIMULATEGROUNDTRUTH) and overlays a bearing ray from each detection
%   in DETECTIONS (see GENERATEDETECTIONS) around the time it occurs.
%   Reuses DRAWSOCCERFIELD for the field itself.
%
%   ANIMATEMULTIROBOTSIM(..., 'PlaybackSpeed', S) plays back at S times
%   real time (default 1). Close the figure window to stop early.

    playbackSpeed = 1;
    for k = 1:2:numel(varargin)
        if strcmp(varargin{k}, 'PlaybackSpeed')
            playbackSpeed = varargin{k+1};
        end
    end

    ax = drawSoccerField(params.Field);

    numRobots = numel(gt.Robots);
    hRobots = cell(numRobots, 1);

    idx = 1;
    for i = 1:numRobots
        robotState = buildRobotState(gt, i, idx, params);
        hRobots{i} = drawRobotSim(ax, robotState, gt.ReportedPoseVariance);
    end
    ballRadius = params.Field.BallRadius * 3; % drawn oversized so it's visible, as in RUNSIMULATOR
    hBall = drawBallGT(ax, gt.Ball.Position(1:2, idx), ballRadius);

    hTitle = title(ax, '', 'Color', [1 1 1]);

    detTimes = [detections.t];
    rayTol = 1.5 * mean(1 ./ [gt.Robots.FrameRate]);
    hRays = gobjects(0);

    fig = ancestor(ax, 'figure');
    frameSkip = max(1, round(1 / (params.Dt * 30))); % aim for ~30 fps playback

    for idx = 1:frameSkip:numel(gt.t)
        if ~isvalid(fig)
            break
        end
        t = gt.t(idx);

        for i = 1:numRobots
            robotState = buildRobotState(gt, i, idx, params);
            updateRobotSim(hRobots{i}, robotState, gt.ReportedPoseVariance);
        end
        updateBallGT(hBall, gt.Ball.Position(1:2, idx), ballRadius);

        delete(hRays(isgraphics(hRays)));
        hRays = gobjects(0);
        active = detections(abs(detTimes - t) <= rayTol);
        for d = active
            endPt = d.rCNn(1:2) + params.MaxRange * d.uNn(1:2);
            if d.isFalsePositive
                col = [0.8 0.1 0.1];
                style = ':';
            else
                col = [0.1 0.8 0.3];
                style = '-';
            end
            hRays(end+1) = plot(ax, [d.rCNn(1), endPt(1)], [d.rCNn(2), endPt(2)], ... %#ok<AGROW>
                'Color', col, 'LineStyle', style, 'LineWidth', 1.25);
        end

        set(hTitle, 'String', sprintf('t = %.2f s', t));
        drawnow limitrate;
        pause(params.Dt * frameSkip / playbackSpeed);
    end

end

function robotState = buildRobotState(gt, i, idx, params)
%BUILDROBOTSTATE True/believed pose snapshot at one time index, for drawing.

    r = gt.Robots(i);

    robotState.TruePose.Position  = r.Position(:, idx).';
    robotState.TruePose.Yaw       = r.Yaw(idx);
    robotState.TruePose.Roll      = r.Roll(idx);
    robotState.TruePose.Pitch     = r.Pitch(idx);
    robotState.TruePose.HeadYaw   = r.HeadYaw(idx);
    robotState.TruePose.HeadPitch = r.HeadPitch(idx);

    robotState.BelievedPose.Position = r.Position(:, idx).' + [r.ErrX(idx), r.ErrY(idx)];
    robotState.BelievedPose.Yaw      = r.Yaw(idx) + r.ErrYaw(idx);

    [robotState.rCNnTrue, robotState.RncTrue] = frames.computeCameraPose(robotState.TruePose, params);

    robotState.Intrinsics = r.Intrinsics;
    robotState.MaxRange   = r.MaxRange;

end
