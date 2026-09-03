function player = stepPlayerWalk(player, cfg, dt)
%STEPPLAYERWALK Advance the player by one smooth, mechatronics-style step.
%   PLAYER = STEPPLAYERWALK(PLAYER, CFG, DT) integrates a unicycle motion
%   model: forward Speed and heading TurnRate are each a damped,
%   noise-driven (Ornstein-Uhlenbeck) process, so they drift smoothly
%   instead of jumping, giving a walking path that curves the way a real
%   mobile robot's does under bounded acceleration/turn-rate control.
%   The look direction (used for the FOV cone) sweeps back and forth
%   independently of the walking heading, like a robot panning its
%   head/camera while it walks. The player reflects off the field
%   boundary from CFG.

    % --- filtered random turn-rate (bounded) ---
    turnNoiseStd = 2.0;             % rad/s^2, std of turn-rate innovation
    turnDamping  = 1.5;             % 1/s, relaxes TurnRate back to 0
    maxTurnRate  = deg2rad(45);

    player.TurnRate = player.TurnRate + dt * (-turnDamping * player.TurnRate + turnNoiseStd * randn());
    player.TurnRate = max(min(player.TurnRate, maxTurnRate), -maxTurnRate);

    % --- filtered random speed (bounded) ---
    speedNoiseStd = 0.5;            % m/s^2, std of speed innovation
    speedDamping  = 1.0;            % 1/s, relaxes Speed back to NominalSpeed

    player.Speed = player.Speed + dt * (-speedDamping * (player.Speed - player.NominalSpeed) + speedNoiseStd * randn());
    player.Speed = max(min(player.Speed, player.MaxSpeed), player.MinSpeed);

    % --- integrate heading and position ---
    player.Heading = player.Heading + player.TurnRate * dt;
    newPos = player.Position + player.Speed * dt * [cos(player.Heading), sin(player.Heading)];

    % --- reflect off the field boundary, turning back inward ---
    halfL = cfg.FieldLength / 2;
    halfW = cfg.FieldWidth / 2;

    if newPos(1) > halfL || newPos(1) < -halfL
        player.Heading  = pi - player.Heading;
        player.TurnRate = -player.TurnRate;
        newPos(1) = max(min(newPos(1), halfL), -halfL);
    end
    if newPos(2) > halfW || newPos(2) < -halfW
        player.Heading  = -player.Heading;
        player.TurnRate = -player.TurnRate;
        newPos(2) = max(min(newPos(2), halfW), -halfW);
    end

    player.Position = newPos;

    % --- independent gaze sweep ---
    player.GazePhase  = player.GazePhase + dt * player.GazeAngularSpeed;
    player.GazeOffset = player.GazeAmplitude * sin(player.GazePhase);

end
