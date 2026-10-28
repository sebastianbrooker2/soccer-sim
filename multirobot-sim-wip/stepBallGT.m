function ball = stepBallGT(ball, cfg, t, dt)
%STEPBALLGT Advance the ball's ground-truth state by one step.
%   BALL = STEPBALLGT(BALL, CFG, T, DT) applies any scheduled kick whose
%   time falls in (T, T+DT], integrates velocity-proportional rolling
%   friction while the ball is on the ground, integrates ballistic
%   flight and bouncing while airborne (only reachable if a kick with a
%   non-zero vertical component was applied, which itself only happens
%   when BALL.ChipKickEnabled), and reflects or clamps at the field
%   boundary from CFG (see FIELDCONFIG) according to BALL.BoundaryMode --
%   the same two boundary conventions offered by STEPPLAYERWALK.

    while ball.NextKick <= numel(ball.Kicks) && ball.Kicks(ball.NextKick).t <= t + dt
        v = ball.Kicks(ball.NextKick).Velocity;
        if ~ball.ChipKickEnabled
            v(3) = 0;
        end
        ball.Velocity = ball.Velocity + v;
        ball.NextKick = ball.NextKick + 1;
    end

    airborne = ball.Position(3) > 0 || ball.Velocity(3) > 0;

    if airborne
        ball.Velocity(3) = ball.Velocity(3) - ball.Gravity * dt;
        ball.Position    = ball.Position + ball.Velocity * dt;
        if ball.Position(3) <= 0
            ball.Position(3) = 0;
            ball.Velocity(3) = -ball.Velocity(3) * ball.BounceRestitution;
            if abs(ball.Velocity(3)) < 0.05
                ball.Velocity(3) = 0;
            end
        end
    else
        ball.Velocity(1:2) = ball.Velocity(1:2) * exp(-ball.FrictionCoeff * dt);
        ball.Position(1:2) = ball.Position(1:2) + ball.Velocity(1:2) * dt;
    end

    halfL = cfg.FieldLength / 2;
    halfW = cfg.FieldWidth / 2;
    reflecting = strcmp(ball.BoundaryMode, 'reflect');

    if ball.Position(1) > halfL || ball.Position(1) < -halfL
        ball.Position(1) = max(min(ball.Position(1), halfL), -halfL);
        ball.Velocity(1)  = -ball.Velocity(1) * reflecting;
    end
    if ball.Position(2) > halfW || ball.Position(2) < -halfW
        ball.Position(2) = max(min(ball.Position(2), halfW), -halfW);
        ball.Velocity(2)  = -ball.Velocity(2) * reflecting;
    end

end
