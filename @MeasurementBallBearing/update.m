function [obj, system] = update(obj, system)
%UPDATE Only fuse this measurement if the ball was actually detected.
%   Reuses COMPUTEBALLDETECTION's FOV-cone + YOLO-style confidence
%   threshold -- the same model the 2D single-player demo uses -- to
%   decide whether the ball was in view of this observer at this event's
%   time (SYSTEM.x_sim, the TRUE ball state, already advanced to this
%   time by EVENT.PROCESS's prior call to SYSTEM.PREDICT). Uses the
%   observer's TRUE pose for this check, deliberately not its believed
%   ObserverPosition/heading -- what a camera can physically see depends
%   on where it actually is, not on what it believes about itself. If
%   not detected, SYSTEM.density is left exactly as the time update left
%   it; only a genuine detection is fused into the estimate.
%
%   A detected measurement can still fail to FUSE: if the prior has
%   drifted far enough from the truth (e.g. a target went unseen by
%   every robot for a while and its uncertainty grew large), the
%   optimiser behind UPDATEMETHOD can run out of iterations without
%   converging, and MEASUREMENT/UPDATE hard-asserts on that with no
%   fallback. Rather than let one bad update crash the whole run, that
%   failure is caught here and the fusion is just skipped (density left
%   as the time update left it, same as "not detected") -- a real onboard
%   fusion filter shouldn't crash because one optimisation step didn't
%   converge either. A warning is still raised so it's visible, not
%   silent.

robot.Position   = obj.TrueObserverPosition.';
robot.Heading    = obj.TrueObserverHeading;
robot.GazeOffset = obj.TrueObserverGazeOffset; % picked up automatically by COMPUTEBALLDETECTION/COMPUTEFOVGEOMETRY's existing isfield(player,'GazeOffset') check
robot.FOV        = obj.ObserverFOV;
robot.Range      = obj.ObserverRange;

ball.Position = system.x_sim(1:2).';

obj.LastDetection = computeBallDetection(robot, ball);

if obj.LastDetection.Detected
    try
        [obj, system] = obj.update@MeasurementGaussianLikelihood(system);
    catch updateError
        warning('MeasurementBallBearing:updateFailed', ...
            't=%.4f RobotID=%d Target=''%s'': %s update did not converge (%s) -- skipping this fusion.', ...
            obj.time, obj.RobotID, obj.TargetName, obj.updateMethod, updateError.message);
        obj.needToSimulate = false;
    end
else
    obj.needToSimulate = false; % Nothing to fuse; don't leave a stale simulate request behind
end

end
