function h = drawRobotSim(ax, robotState, reportedPosVar)
%DRAWROBOTSIM Draw one robot's true and believed state for the top-down
%   multi-robot animation.
%   H = DRAWROBOTSIM(AX, ROBOTSTATE, REPORTEDPOSVAR) draws the TRUE
%   position/heading, the TRUE camera's FOV wedge (see
%   COMPUTECAMERAFOVWEDGE), and the BELIEVED position with a 2-sigma
%   position covariance ellipse -- reusing COMPUTECOVARIANCEELLIPSE from
%   the existing 2D player viz, since it only needs a struct with
%   Position/Covariance fields.
%   ROBOTSTATE is a struct with fields TruePose, BelievedPose, rCNnTrue,
%   RncTrue, Intrinsics, MaxRange (see ANIMATEMULTIROBOTSIM).

    trueColor     = [0.95 0.35 0.05];
    believedColor = [0.10 0.45 0.90];
    fovColor      = [1.00 0.90 0.10];

    [fovX, fovY] = computeCameraFOVWedge(robotState.rCNnTrue, robotState.RncTrue, ...
        robotState.Intrinsics, robotState.MaxRange);
    h.FOVPatch = patch(ax, fovX, fovY, fovColor, 'FaceAlpha', 0.10, 'EdgeColor', 'none');

    h.HeadingLen = 0.6;
    h.TrueMarker = plot(ax, robotState.TruePose.Position(1), robotState.TruePose.Position(2), 'o', ...
        'MarkerFaceColor', trueColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
    h.TrueHeading = plot(ax, ...
        [robotState.TruePose.Position(1), robotState.TruePose.Position(1) + h.HeadingLen*cos(robotState.TruePose.Yaw)], ...
        [robotState.TruePose.Position(2), robotState.TruePose.Position(2) + h.HeadingLen*sin(robotState.TruePose.Yaw)], ...
        'Color', trueColor, 'LineWidth', 2);

    believedPlayer.Position   = robotState.BelievedPose.Position;
    believedPlayer.Covariance = diag(reportedPosVar(1:2));
    [ellX, ellY] = computeCovarianceEllipse(believedPlayer);
    h.BelievedEllipse = plot(ax, ellX, ellY, 'Color', believedColor, 'LineWidth', 1.5);
    h.BelievedMarker = plot(ax, robotState.BelievedPose.Position(1), robotState.BelievedPose.Position(2), '^', ...
        'MarkerFaceColor', believedColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);

end
