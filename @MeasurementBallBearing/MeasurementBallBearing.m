classdef MeasurementBallBearing < MeasurementGaussianLikelihood
%MEASUREMENTBALLBEARING Bearing (+ rough assumed depth) measurement from
%   one robot observer.
%   y = (ball_position - ObserverPosition) / norm(ball_position -
%   ObserverPosition) -- a 2D UNIT BEARING VECTOR in the FIELD frame, not
%   a scalar angle (not relative to the observer's own heading either --
%   TrueObserverHeading/ObserverFOV/ObserverRange below are only used,
%   via COMPUTEBALLDETECTION, to decide whether the ball was in view at
%   all; same role as in MEASUREMENTBALLPOSITION). This mirrors how a
%   real detector reports it (a pixel/direction vector, not an angle)
%   and, unlike atan2(dy,dx), has no periodic branch cut to wrap around.
%
%   Noise combines a fixed DETECTIONNOISESTD, the observer's own heading
%   uncertainty (OBSERVERHEADINGVARIANCE), and its position uncertainty
%   (OBSERVERCOVARIANCE) projected onto the TANGENTIAL direction
%   (perpendicular to the line of sight) and scaled by 1/range -- see
%   NOISEDENSITY. Heading and position errors both only ever displace the
%   bearing tangentially (rotating or shifting sideways never moves you
%   towards/away from the target), but they scale with range
%   differently: a heading error looks just as bad at any range (an
%   observer that doesn't know which way it's facing is equally wrong
%   about a near or far target), whereas the same absolute position
%   error matters a lot up close and is negligible far away -- that's
%   why only the position term is divided by range^2. That range
%   dependence, and the fact that a single measurement only constrains
%   the tangential direction well and the depth direction only roughly
%   (see NOISEDENSITY's assumedDepthStd), is why this measurement's
%   covariance in the field frame comes out elongated along the range
%   direction rather than circular, and why triangulating range
%   precisely still needs a second observer at a different angle (or the
%   process model's prior).
%
%   A unit vector has only 1 true degree of freedom (it's a point on a
%   circle) despite being 2 numbers, so on its own the noise built from
%   the tangential terms above would be rank-1 (singular). NOISEDENSITY
%   adds a variance along the bearing direction itself to fix that --
%   not a token numerical fudge factor, but a deliberate, if rough, ASSUMED
%   depth uncertainty standing in for the kind of imprecise range cue a
%   real detector can often get for free (e.g. from the target's known
%   size in pixels).
%
%   The observer/robot itself is not part of SYSTEMBALL's state or
%   dynamics -- only its (Gaussian) position estimate is known, and is
%   carried here per-measurement (one instance per robot per detection),
%   the same way MEASUREMENTRADAR carries its sensor location, but
%   per-instance and uncertain instead of a fixed class constant.

    properties
        ObserverPosition (2, 1) double = zeros(2, 1)   % Believed observer/robot position in the field frame, m
        ObserverCovariance (2, 2) double = zeros(2, 2) % Observer's own position covariance, m^2
        ObserverHeadingVariance (1, 1) double = 0      % Observer's own heading-belief variance, rad^2 -- see NOISEDENSITY
        DetectionNoiseStd (1, 1) double = deg2rad(1)   % Tangential noise std of the raw bearing detection (rad, since a unit bearing's tangential displacement for a small angle equals the angle itself)
        RangeNoiseStd (1, 1) double = nan              % Fixed std of the range measurement, m. NaN (default) = bearing-only, y is the 2x1 unit vector and NOISEDENSITY uses the assumed-depth term; finite = range-bearing, y = [unit bearing; range] (3x1) -- see PREDICT/NOISEDENSITY
        RobotID (1, 1) double = 0                      % Which robot/observer this measurement came from (bookkeeping only)
        TargetName (1, :) char = ''                     % Which tracked object this is a measurement OF, e.g. 'ball'/'opp1' (bookkeeping only -- the class/prediction math never reads it). Real detections would need an actual data-association step to assign this when several targets could look alike; here it's assumed known.

        AlreadyDetected (1, 1) logical = false         % Skip UPDATE's own COMPUTEBALLDETECTION re-check and trust LASTDETECTION as already set. Used by a data-association front end that has already decided detection-worthiness against ground truth once, when generating this measurement's Y -- re-running COMPUTEBALLDETECTION here would draw a fresh random confidence jitter and could inconsistently flip that outcome.

        % FOV-cone/YOLO-threshold detection gate (see UPDATE), reusing
        % COMPUTEBALLDETECTION's model from the 2D single-player demo.
        % Deliberately the TRUE pose, not the believed one above -- what
        % a camera can physically see depends on where it actually is,
        % not on what the robot believes about itself.
        TrueObserverPosition (2, 1) double = zeros(2, 1) % Observer's true position in the field frame, m (ground truth, for the FOV gate only)
        TrueObserverHeading (1, 1) double = 0            % Observer's true look direction, rad (ground truth, for the FOV gate only)
        TrueObserverGazeOffset (1, 1) double = 0         % Observer's true head-pan offset from TrueObserverHeading, rad (ground truth, for the FOV gate only -- same GazeOffset concept as CREATEPLAYER/COMPUTEFOVGEOMETRY)
        ObserverFOV     (1, 1) double = deg2rad(130)  % Observer's field-of-view, rad (matches CREATEPLAYER's default)
        ObserverRange   (1, 1) double = 10            % Observer's max detection range, m

        LastDetection = struct('Distance', nan, 'AngleError', nan, 'Confidence', 0, 'Detected', false) % Bookkeeping: COMPUTEBALLDETECTION's output for this event
    end

    methods (Static)
        s = getProcessString()
    end

    methods
        [h, dhdx, d2hdx2] = predict(obj, x, system)     % Return h(x) and derivatives w.r.t. x
        out = noiseDensity(obj, system)                 % Return noise density p(v)
    end

    methods (Access = protected)
        [obj, system] = update(obj, system)
    end
end
