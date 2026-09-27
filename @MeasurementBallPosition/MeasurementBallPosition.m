classdef MeasurementBallPosition < MeasurementGaussianLikelihood
%MEASUREMENTBALLPOSITION Relative-position ball measurement from one
%   robot observer.
%   y = ball_position - ObserverPosition + noise, where noise combines
%   the detector's own DETECTIONNOISECOVARIANCE with the observer's own
%   OBSERVERCOVARIANCE (uncertainty in the observer's believed position),
%   since an error in the observer's own position propagates directly
%   into the relative-position measurement.
%
%   The observer/robot itself is not part of SYSTEMBALL's state or
%   dynamics -- only its (Gaussian) position estimate is known, and is
%   carried here per-measurement (one instance per robot per detection),
%   the same way MEASUREMENTRADAR carries its sensor location, but
%   per-instance and uncertain instead of a fixed class constant.

    properties
        ObserverPosition (2, 1) double = zeros(2, 1)          % Believed observer/robot position in the field frame, m
        ObserverCovariance (2, 2) double = zeros(2, 2)        % Observer's own position covariance, m^2
        DetectionNoiseCovariance (2, 2) double = 0.01*eye(2)  % Covariance of the raw relative-position detection, m^2
        RobotID (1, 1) double = 0                             % Which robot/observer this measurement came from (bookkeeping only)

        % FOV-cone/YOLO-threshold detection gate (see UPDATE), reusing
        % COMPUTEBALLDETECTION's model from the 2D single-player demo.
        % This is about whether the ball was in view at all, and is
        % independent of ObserverPosition/ObserverCovariance above, which
        % only shape the measurement noise once a detection does occur.
        ObserverHeading (1, 1) double = 0             % Observer's fixed look direction, rad (0 = +x axis)
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
