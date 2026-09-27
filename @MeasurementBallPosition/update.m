function [obj, system] = update(obj, system)
%UPDATE Only fuse this measurement if the ball was actually detected.
%   Reuses COMPUTEBALLDETECTION's FOV-cone + YOLO-style confidence
%   threshold -- the same model the 2D single-player demo uses -- to
%   decide whether the ball was in view of this observer at this event's
%   time (SYSTEM.x_sim, the TRUE ball state, already advanced to this
%   time by EVENT.PROCESS's prior call to SYSTEM.PREDICT). If not
%   detected, SYSTEM.density is left exactly as the time update left it;
%   only a genuine detection is fused into the estimate.

robot.Position = obj.ObserverPosition.';
robot.Heading  = obj.ObserverHeading;
robot.FOV      = obj.ObserverFOV;
robot.Range    = obj.ObserverRange;

ball.Position = system.x_sim(1:2).';

obj.LastDetection = computeBallDetection(robot, ball);

% --- DEBUG: is the density already broken *before* this update touches it? ---
XiBefore = system.density.sqrtInfoMat();
badBefore = any(~isfinite(XiBefore(:)));
if badBefore
    fprintf(2, '[DEBUG] t=%.4f RobotID=%d: density.Xi already non-finite BEFORE this update (predict, or an earlier update, corrupted it)\n', ...
        obj.time, obj.RobotID);
end

if obj.LastDetection.Detected
    [obj, system] = obj.update@MeasurementGaussianLikelihood(system);

    % --- DEBUG: did *this* fusion introduce the corruption? ---
    XiAfter = system.density.sqrtInfoMat();
    if ~badBefore && any(~isfinite(XiAfter(:)))
        fprintf(2, ['[DEBUG] t=%.4f RobotID=%d: THIS update just introduced non-finite Xi. ' ...
            'Confidence=%.4f Distance=%.4f AngleError=%.4f y=[%s] ObserverPosition=[%s]\n'], ...
            obj.time, obj.RobotID, obj.LastDetection.Confidence, obj.LastDetection.Distance, ...
            obj.LastDetection.AngleError, sprintf('%.4f ', obj.y), sprintf('%.4f ', obj.ObserverPosition));
        fprintf(2, '         Xi before:\n'); disp(XiBefore);
        fprintf(2, '         Xi after:\n');  disp(XiAfter);
    end
else
    obj.needToSimulate = false; % Nothing to fuse; don't leave a stale simulate request behind
end

end
