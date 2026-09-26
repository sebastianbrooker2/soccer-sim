function ball = createBall(position, cfg)
%CREATEBALL Create a stationary ball on the field.
%   BALL = CREATEBALL(POSITION, CFG) places a stationary ball at POSITION
%   ([x y], metres) with radius CFG.BallRadius (see FIELDCONFIG).

    if nargin < 2
        cfg = fieldConfig();
    end

    ball.Position = position(:)';
    ball.Radius   = cfg.BallRadius;

end
