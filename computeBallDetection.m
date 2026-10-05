function det = computeBallDetection(player, ball)
%COMPUTEBALLDETECTION Simulate a YOLO-style detection of a stationary ball.
%   DET = COMPUTEBALLDETECTION(PLAYER, BALL) checks whether BALL falls
%   inside PLAYER's field-of-view wedge (look direction PLAYER.Heading +
%   PLAYER.GazeOffset, half-angle PLAYER.FOV/2, out to PLAYER.Range), and
%   if so, simulates a detector confidence score the way a real
%   vision-based ball detector behaves: it starts near-certain and loses
%   confidence the farther the ball is and the less centred it is in the
%   FOV, with a little frame-to-frame jitter, and clipped so nothing
%   below the detector's 20% confidence cutoff is ever reported as a
%   detection.
%
%   Returns a struct:
%     Distance    distance from player to ball, metres
%     AngleError  bearing to ball minus look direction, wrapped to
%                 [-pi, pi] (0 = dead centre of FOV)
%     Confidence  simulated confidence in [0, 1]; 0 whenever the ball is
%                 outside the FOV wedge or beyond Range
%     Detected    true only when Confidence >= 0.20 (the detector's
%                 cutoff) -- below that, a real detector simply would
%                 not report a box, so nothing is "detected" here either

    CONFCUTOFF = 0.20;

    if isfield(player, 'GazeOffset')
        lookHeading = player.Heading + player.GazeOffset;
    else
        lookHeading = player.Heading;
    end

    delta   = ball.Position - player.Position;
    d       = norm(delta);
    bearing = atan2(delta(2), delta(1));
    angErr  = atan2(sin(bearing - lookHeading), cos(bearing - lookHeading));

    halfFOV = player.FOV / 2;
    inRange = d <= player.Range;
    inFOV   = abs(angErr) <= halfFOV;

    det.Distance   = d;
    det.AngleError = angErr;

    if ~inRange || ~inFOV
        det.Confidence = 0;
        det.Detected   = false;
        return
    end

    rangePenalty = d / player.Range;              % 0 close .. 1 at max range
    anglePenalty = abs(angErr) / halfFOV;         % 0 centred .. 1 at FOV edge

    % Start near-certain and dock confidence for range and off-centre
    % separately (not multiplicatively), so most of the FOV wedge stays
    % above the cutoff and only the far/edge extremes drop out.
    conf = 0.99 - 0.35 * rangePenalty - 0.35 * anglePenalty;
    conf = conf + 0.02 * randn();                 % small detector jitter, like real per-frame noise
    conf = min(max(conf, 0), 0.99);               % a real detector is never 100% sure

    det.Confidence = conf;
    det.Detected   = conf >= CONFCUTOFF;

end
