function robot = stepRobotGT(robot, params, ballPosition, dt)
%STEPROBOTGT Advance one robot's true and believed state by one step.
%   ROBOT = STEPROBOTGT(ROBOT, PARAMS, BALLPOSITION, DT)
%   BALLPOSITION is the ball's TRUE [x y z] position, m.
%
%   Body motion: a smoothed unicycle controller (OU-filtered Speed and
%   TurnRate, as in STEPPLAYERWALK) steers towards a target point, either
%   an offset around the ball or the next patrol waypoint, chosen by
%   PARAMS.RobotBehavior. A periodic roll/pitch wobble is layered on top
%   to represent the walking gait.
%
%   Head motion: a look-at controller computes the head angles that
%   would centre the TRUE ball in the TRUE camera, and the head slews
%   towards that target (rate- and gimbal-limited) whenever the ball is
%   plausibly trackable; otherwise the head sweeps a scan pattern.
%
%   Believed-pose error: ROBOT.ErrX/ErrY/ErrYaw/ErrHeadPitch are advanced
%   as independent OU processes the robot itself does not know about --
%   see SIMPARAMS for why body roll/pitch is excluded.

    % --- target point for the walking controller ---
    if strcmp(params.RobotBehavior, 'waypoints')
        target = robot.Waypoints(robot.WaypointIndex, :);
        if norm(target - robot.Position) < params.WaypointArriveRadius
            robot.WaypointIndex = mod(robot.WaypointIndex, size(robot.Waypoints, 1)) + 1;
            target = robot.Waypoints(robot.WaypointIndex, :);
        end
    else % 'walkToBall'
        target = ballPosition(1:2) + params.WalkToBallOffsetRadius * ...
            [cos(robot.TargetAngleOffset), sin(robot.TargetAngleOffset)];
    end

    % --- smoothed unicycle walking controller ---
    desiredHeading = atan2(target(2) - robot.Position(2), target(1) - robot.Position(1));
    headingError   = atan2(sin(desiredHeading - robot.Yaw), cos(desiredHeading - robot.Yaw));

    desiredTurnRate = max(min(params.HeadingKp * headingError, params.MaxTurnRate), -params.MaxTurnRate);
    robot.TurnRate  = robot.TurnRate + dt * (-params.TurnDamping * (robot.TurnRate - desiredTurnRate) ...
        + params.TurnNoiseStd * sqrt(dt) * randn());
    robot.TurnRate  = max(min(robot.TurnRate, params.MaxTurnRate), -params.MaxTurnRate);

    desiredSpeed = params.NominalSpeed * max(1 - min(1, abs(headingError)/pi), 0.15);
    robot.Speed  = robot.Speed + dt * (-params.SpeedDamping * (robot.Speed - desiredSpeed) ...
        + params.SpeedNoiseStd * sqrt(dt) * randn());
    robot.Speed  = max(min(robot.Speed, params.MaxSpeed), params.MinSpeed);

    robot.Yaw = robot.Yaw + robot.TurnRate * dt;
    newPos    = robot.Position + robot.Speed * dt * [cos(robot.Yaw), sin(robot.Yaw)];

    % --- reflect off the field boundary, as in STEPPLAYERWALK ---
    halfL = params.Field.FieldLength / 2;
    halfW = params.Field.FieldWidth / 2;
    if newPos(1) > halfL || newPos(1) < -halfL
        robot.Yaw      = pi - robot.Yaw;
        robot.TurnRate = -robot.TurnRate;
        newPos(1)      = max(min(newPos(1), halfL), -halfL);
    end
    if newPos(2) > halfW || newPos(2) < -halfW
        robot.Yaw      = -robot.Yaw;
        robot.TurnRate = -robot.TurnRate;
        newPos(2)      = max(min(newPos(2), halfW), -halfW);
    end
    robot.Position = newPos;

    % --- periodic walking wobble ---
    robot.WobbleRollPhase  = robot.WobbleRollPhase + dt * 2*pi*params.WobbleRollFrequency;
    robot.WobblePitchPhase = robot.WobblePitchPhase + dt * 2*pi*params.WobblePitchFrequency;
    robot.Roll  = params.WobbleRollAmplitude * sin(robot.WobbleRollPhase);
    robot.Pitch = params.WobblePitchAmplitude * sin(robot.WobblePitchPhase);

    % --- head: look-at controller vs. scan, using the TRUE body attitude ---
    Rnb        = frames.rotz(robot.Yaw) * frames.roty(robot.Pitch) * frames.rotx(robot.Roll);
    headJoint  = [robot.Position(:); params.HeadHeight];
    relN       = ballPosition(:) - headJoint;
    relBody    = Rnb.' * relN;
    horizRange = hypot(relBody(1), relBody(2));

    desiredHeadYaw   = atan2(relBody(2), relBody(1));
    desiredHeadPitch = atan2(-relBody(3), horizRange);

    trackable = norm(relN) <= params.MaxRange * params.HeadTrackMargin ...
        && abs(desiredHeadYaw) <= params.HeadYawLimit * params.HeadTrackMargin;

    if trackable
        targetHeadYaw   = desiredHeadYaw;
        targetHeadPitch = desiredHeadPitch;
    else
        robot.ScanPhase = robot.ScanPhase + dt * 2*pi*params.HeadScanFrequency;
        targetHeadYaw   = params.HeadScanAmplitude * sin(robot.ScanPhase);
        targetHeadPitch = params.HeadScanPitch;
    end

    targetHeadYaw   = max(min(targetHeadYaw, params.HeadYawLimit), -params.HeadYawLimit);
    targetHeadPitch = max(min(targetHeadPitch, params.HeadPitchLimit(2)), params.HeadPitchLimit(1));

    robot.HeadYaw   = slewToward(robot.HeadYaw, targetHeadYaw, params.MaxHeadYawRate, dt);
    robot.HeadPitch = slewToward(robot.HeadPitch, targetHeadPitch, params.MaxHeadPitchRate, dt);

    % --- believed-pose error: independent OU biases (Euler-Maruyama) ---
    robot.ErrX = robot.ErrX + dt*(-params.LocErrorDamping(1)*robot.ErrX) ...
        + params.LocErrorNoiseStd(1)*sqrt(dt)*randn();
    robot.ErrY = robot.ErrY + dt*(-params.LocErrorDamping(2)*robot.ErrY) ...
        + params.LocErrorNoiseStd(2)*sqrt(dt)*randn();
    robot.ErrYaw = robot.ErrYaw + dt*(-params.LocErrorDamping(3)*robot.ErrYaw) ...
        + params.LocErrorNoiseStd(3)*sqrt(dt)*randn();
    robot.ErrHeadPitch = robot.ErrHeadPitch + dt*(-params.LocErrorDamping(4)*robot.ErrHeadPitch) ...
        + params.LocErrorNoiseStd(4)*sqrt(dt)*randn();

end

function y = slewToward(y0, target, maxRate, dt)
%SLEWTOWARD Move angle Y0 towards TARGET by at most MAXRATE*DT, shortest way round.
    err  = atan2(sin(target - y0), cos(target - y0));
    step = max(min(err, maxRate*dt), -maxRate*dt);
    y    = y0 + step;
end
