function player = createPlayer(position, heading, covariance)
%CREATEPLAYER Create a soccer player with a smooth walking motion model.
%   PLAYER = CREATEPLAYER(POSITION, HEADING, COVARIANCE)
%   POSITION   [x y] location on the field, in metres.
%   HEADING    initial walking direction, in radians (0 = +x axis).
%   COVARIANCE 2x2 position covariance matrix (metres^2).
%
%   PLAYER also carries FOV (130 degrees) and Range (10 m), used to draw
%   its field of view, plus the internal state STEPPLAYERWALK uses to
%   move it: a filtered forward Speed/TurnRate (so the walking path
%   curves smoothly instead of jumping randomly) and an independent
%   GazeOffset that sweeps the look direction back and forth relative to
%   the body heading, like a robot panning its head/camera while it
%   walks.

    if nargin < 3
        covariance = [0.3 0; 0 0.3];
    end
    if nargin < 2
        heading = 0;
    end

    player.Position   = position(:)';
    player.Heading    = heading;
    player.Covariance = covariance;
    player.FOV        = deg2rad(130);
    player.Range      = 10;

    % --- walking motion state (unicycle model, smoothed) ---
    player.Speed        = 0.8;             % current forward speed, m/s
    player.NominalSpeed = 0.8;             % speed the walk relaxes towards
    player.MinSpeed     = 0.2;
    player.MaxSpeed     = 1.4;
    player.TurnRate     = 0;               % current heading rate, rad/s

    % --- independent look/gaze sweep ---
    player.GazeOffset      = 0;                     % rad, relative to Heading
    player.GazePhase       = 2*pi*rand();
    player.GazeAmplitude   = deg2rad(45);
    player.GazeAngularSpeed = 2*pi / 4;    % rad/s, ~4 s per full sweep cycle

end
