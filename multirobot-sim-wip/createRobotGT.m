function robot = createRobotGT(id, initialPosition, initialYaw, params)
%CREATEROBOTGT Initialise one robot's true and believed ground-truth state.
%   ROBOT = CREATEROBOTGT(ID, INITIALPOSITION, INITIALYAW, PARAMS)
%   INITIALPOSITION is [x y], m; INITIALYAW is rad.
%
%   Sets up:
%    - a true body pose (x, y, yaw) plus the smoothed OU walking
%      controller (Speed, TurnRate), in the style of STEPPLAYERWALK, and a
%      periodic roll/pitch walking wobble (see SIMPARAMS);
%    - a head (HeadYaw, HeadPitch) that tracks the ball or scans;
%    - a believed-pose error state (ErrX, ErrY, ErrYaw, ErrHeadPitch), an
%      OU bias process the robot itself does not know about (see
%      SIMPARAMS);
%    - this robot's camera frame rate, phase offset and intrinsics.

    robot.ID = id;

    robot.Position = initialPosition(:)';
    robot.Yaw      = initialYaw;
    robot.Roll     = 0;
    robot.Pitch    = 0;

    robot.Speed        = params.NominalSpeed;
    robot.NominalSpeed = params.NominalSpeed;
    robot.MinSpeed     = params.MinSpeed;
    robot.MaxSpeed     = params.MaxSpeed;
    robot.TurnRate     = 0;

    % Even spread around the ball for the 'walkToBall' behaviour.
    robot.TargetAngleOffset = 2*pi*(id - 1) / max(params.NumRobots, 1);

    % Simple rectangular patrol loop for the 'waypoints' behaviour,
    % inset from the boundary and phase-shifted per robot.
    halfL = params.Field.FieldLength/2 - 1.5;
    halfW = params.Field.FieldWidth/2 - 1.5;
    loop  = [-halfL -halfW; halfL -halfW; halfL halfW; -halfL halfW];
    robot.Waypoints     = circshift(loop, -mod(id - 1, size(loop, 1)));
    robot.WaypointIndex = 1;

    robot.WobbleRollPhase  = 2*pi*rand();
    robot.WobblePitchPhase = 2*pi*rand();

    robot.HeadYaw   = 0;
    robot.HeadPitch = 0;
    robot.ScanPhase = 2*pi*rand();

    robot.ErrX         = 0;
    robot.ErrY         = 0;
    robot.ErrYaw       = 0;
    robot.ErrHeadPitch = 0;

    robot.FrameRate = max(5, params.FrameRateBase + params.FrameRateJitterStd*randn());
    if params.RandomFramePhase
        robot.FramePhase = rand() / params.FrameRateBase;
    else
        robot.FramePhase = 0;
    end

    robot.Intrinsics = struct( ...
        'ImageWidth', params.ImageWidth, 'ImageHeight', params.ImageHeight, ...
        'fx', params.fx, 'fy', params.fy, 'cx', params.cx, 'cy', params.cy, ...
        'DistortionCoeffs', params.DistortionCoeffs);
    robot.MaxRange = params.MaxRange;

end
