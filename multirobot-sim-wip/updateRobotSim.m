function updateRobotSim(h, robotState, reportedPosVar)
%UPDATEROBOTSIM Move an already-drawn robot's graphics (see DRAWROBOTSIM)
%   to a new state, without recreating any graphics objects.

    [fovX, fovY] = computeCameraFOVWedge(robotState.rCNnTrue, robotState.RncTrue, ...
        robotState.Intrinsics, robotState.MaxRange);
    set(h.FOVPatch, 'XData', fovX, 'YData', fovY);

    set(h.TrueMarker, 'XData', robotState.TruePose.Position(1), 'YData', robotState.TruePose.Position(2));
    set(h.TrueHeading, ...
        'XData', [robotState.TruePose.Position(1), robotState.TruePose.Position(1) + h.HeadingLen*cos(robotState.TruePose.Yaw)], ...
        'YData', [robotState.TruePose.Position(2), robotState.TruePose.Position(2) + h.HeadingLen*sin(robotState.TruePose.Yaw)]);

    believedPlayer.Position   = robotState.BelievedPose.Position;
    believedPlayer.Covariance = diag(reportedPosVar(1:2));
    [ellX, ellY] = computeCovarianceEllipse(believedPlayer);
    set(h.BelievedEllipse, 'XData', ellX, 'YData', ellY);
    set(h.BelievedMarker, 'XData', robotState.BelievedPose.Position(1), 'YData', robotState.BelievedPose.Position(2));

end
