function ball = createBallGT(params)
%CREATEBALLGT Initialise the ball's 3D ground-truth state.
%   BALL = CREATEBALLGT(PARAMS) places the ball at
%   PARAMS.Ball.InitialPosition (on the ground, z = 0) with zero
%   velocity, and copies the dynamics parameters and kick schedule from
%   PARAMS.Ball (see SIMPARAMS). Reuses FIELDCONFIG's BallRadius via
%   PARAMS.Field.

    ball.Position = [params.Ball.InitialPosition(:)', 0]; % [x y z], m
    ball.Velocity = [0 0 0];
    ball.Radius   = params.Field.BallRadius;

    ball.FrictionCoeff     = params.Ball.FrictionCoeff;
    ball.ChipKickEnabled   = params.Ball.ChipKickEnabled;
    ball.Gravity           = params.Ball.Gravity;
    ball.BounceRestitution = params.Ball.BounceRestitution;
    ball.BoundaryMode      = params.Ball.BoundaryMode;

    kicks = params.Ball.Kicks;
    if params.Ball.ChipKickEnabled && isfield(params.Ball, 'ChipKicks')
        kicks = [kicks, params.Ball.ChipKicks];
    end
    [~, order] = sort([kicks.t]);
    ball.Kicks    = kicks(order);
    ball.NextKick = 1;

end
