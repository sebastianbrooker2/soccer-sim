function gt = simulateGroundTruth(params)
%SIMULATEGROUNDTRUTH Run the ball + multi-robot ground-truth simulation.
%   GT = SIMULATEGROUNDTRUTH(PARAMS) integrates the ball (see
%   CREATEBALLGT/STEPBALLGT) and every robot (see
%   CREATEROBOTGT/STEPROBOTGT) over PARAMS.Duration at PARAMS.Dt, and
%   returns the full time history of true and believed state. Kept
%   separate from measurement generation (see GENERATEDETECTIONS) so the
%   ground truth can be saved and used to evaluate a fusion filter later.
%
%   Seeds the RNG from PARAMS.Seed so runs are reproducible.

    rng(params.Seed);

    t = 0:params.Dt:params.Duration;
    N = numel(t);

    ball = createBallGT(params);

    robots = repmat(struct(), params.NumRobots, 1);
    for i = 1:params.NumRobots
        angle0   = 2*pi*(i-1)/params.NumRobots;
        startPos = 0.7 * [params.Field.FieldLength/2, params.Field.FieldWidth/2] .* [cos(angle0), sin(angle0)];
        robots(i) = createRobotGT(i, startPos, angle0 + pi, params);
    end

    gt.t      = t;
    gt.Params = params;

    gt.Ball.Position = zeros(3, N);
    gt.Ball.Velocity = zeros(3, N);

    for i = 1:params.NumRobots
        gt.Robots(i).ID           = robots(i).ID;
        gt.Robots(i).Position     = zeros(2, N);
        gt.Robots(i).Yaw          = zeros(1, N);
        gt.Robots(i).Roll         = zeros(1, N);
        gt.Robots(i).Pitch        = zeros(1, N);
        gt.Robots(i).HeadYaw      = zeros(1, N);
        gt.Robots(i).HeadPitch    = zeros(1, N);
        gt.Robots(i).ErrX         = zeros(1, N);
        gt.Robots(i).ErrY         = zeros(1, N);
        gt.Robots(i).ErrYaw       = zeros(1, N);
        gt.Robots(i).ErrHeadPitch = zeros(1, N);
        gt.Robots(i).Intrinsics   = robots(i).Intrinsics;
        gt.Robots(i).MaxRange     = robots(i).MaxRange;
        gt.Robots(i).FrameRate    = robots(i).FrameRate;
        gt.Robots(i).FramePhase   = robots(i).FramePhase;
    end

    for k = 1:N
        gt.Ball.Position(:, k) = ball.Position(:);
        gt.Ball.Velocity(:, k) = ball.Velocity(:);

        for i = 1:params.NumRobots
            gt.Robots(i).Position(:, k)  = robots(i).Position(:);
            gt.Robots(i).Yaw(k)          = robots(i).Yaw;
            gt.Robots(i).Roll(k)         = robots(i).Roll;
            gt.Robots(i).Pitch(k)        = robots(i).Pitch;
            gt.Robots(i).HeadYaw(k)      = robots(i).HeadYaw;
            gt.Robots(i).HeadPitch(k)    = robots(i).HeadPitch;
            gt.Robots(i).ErrX(k)         = robots(i).ErrX;
            gt.Robots(i).ErrY(k)         = robots(i).ErrY;
            gt.Robots(i).ErrYaw(k)       = robots(i).ErrYaw;
            gt.Robots(i).ErrHeadPitch(k) = robots(i).ErrHeadPitch;
        end

        if k < N
            ball = stepBallGT(ball, params.Field, t(k), params.Dt);
            for i = 1:params.NumRobots
                robots(i) = stepRobotGT(robots(i), params, ball.Position, params.Dt);
            end
        end
    end

    % Stationary OU variance per axis is Std^2/(2*Damping); the reported
    % covariance robots carry is a constant scaling of this "true" variance
    % (see SIMPARAMS.ReportedCovScale), for [x y yaw headPitch].
    stationaryVar = (params.LocErrorNoiseStd.^2) ./ (2*params.LocErrorDamping);
    gt.ReportedPoseVariance = params.ReportedCovScale * stationaryVar;

end
