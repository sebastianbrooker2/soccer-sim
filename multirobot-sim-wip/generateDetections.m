function detections = generateDetections(gt, params)
%GENERATEDETECTIONS Simulate per-frame single-camera ball detections.
%   DETECTIONS = GENERATEDETECTIONS(GT, PARAMS) walks GT (the ground
%   truth produced by SIMULATEGROUNDTRUTH, kept separate from this
%   function) at each robot's own camera frame rate and phase offset,
%   and for each frame decides whether the ball is detected: in front of
%   the camera, inside the image, within PARAMS.MaxRange, unoccluded
%   (PARAMS.OcclusionEnabled, modelling other robots as cylinders via
%   CHECKOCCLUSION), and passing a range-dependent detection probability
%   (COMPUTEDETECTIONPROBABILITY).
%
%   Every reported quantity (pixel, camera pose, bearing) is computed
%   from the robot's BELIEVED pose, not its true one -- see SIMPARAMS for
%   how the two differ. Only the detection decision itself, and the true
%   pixel that noise is added to, use the true geometry.
%
%   DETECTIONS is a struct array sorted by time t, each element with
%   fields t, robotID, pixel, rCNn, Rnc, uNn, P_bearing, S_bearing (the
%   upper-triangular square-root of P_bearing, via GaussianInfo, to match
%   the square-root information filter this simulator feeds) and
%   isFalsePositive.
%
%   Note on P_bearing: a unit bearing has 2 true degrees of freedom, so
%   the pixel- and orientation-noise contributions built below only ever
%   perturb DET.uNn within its tangent plane, making the raw 3x3 sum
%   singular. PARAMS.BearingRadialVariance adds a tiny variance along
%   uNn itself so GaussianInfo.fromMoment (which needs a positive
%   definite matrix) can be used.
%
%   Also note: the robot's BELIEVED x/y position uncertainty does not
%   appear in P_bearing at all -- a translation of the camera does not
%   change the *direction* of a unit bearing, only where the fusion
%   filter thinks the ray starts (rCNn). Only the BELIEVED orientation
%   (yaw, head pitch) contributes.

    rng(params.Seed + 1); % independent of, but reproducible alongside, SIMULATEGROUNDTRUTH

    detections = struct('t', {}, 'robotID', {}, 'pixel', {}, 'rCNn', {}, 'Rnc', {}, ...
        'uNn', {}, 'P_bearing', {}, 'S_bearing', {}, 'isFalsePositive', {});

    numRobots = numel(gt.Robots);

    for i = 1:numRobots
        robot = gt.Robots(i);
        frameTimes = robot.FramePhase:(1/robot.FrameRate):gt.t(end);

        for tk = frameTimes
            idx = round((tk - gt.t(1)) / params.Dt) + 1;
            idx = max(min(idx, numel(gt.t)), 1);
            t = gt.t(idx);

            truePose = poseAtIndex(gt, i, idx, false);
            [rCNnTrue, RncTrue] = frames.computeCameraPose(truePose, params);

            ballPos = gt.Ball.Position(:, idx);
            relN    = ballPos - rCNnTrue;
            relC    = RncTrue.' * relN;
            range   = norm(relN);

            believedPose = poseAtIndex(gt, i, idx, true);

            if relC(3) > 0
                truePixel = projectPoint(relC, robot.Intrinsics);
                insideImage = truePixel(1) >= 0 && truePixel(1) <= robot.Intrinsics.ImageWidth ...
                    && truePixel(2) >= 0 && truePixel(2) <= robot.Intrinsics.ImageHeight;
                inRange = range <= robot.MaxRange;

                occluded = false;
                if params.OcclusionEnabled && (insideImage && inRange)
                    otherPositions = zeros(numRobots - 1, 2);
                    n = 0;
                    for j = 1:numRobots
                        if j ~= i
                            n = n + 1;
                            otherPositions(n, :) = gt.Robots(j).Position(:, idx).';
                        end
                    end
                    occluded = checkOcclusion(rCNnTrue, ballPos, otherPositions, ...
                        params.RobotCylinderRadius, params.RobotCylinderHeight);
                end

                pd = computeDetectionProbability(range, robot.MaxRange, params);

                if insideImage && inRange && ~occluded && rand() <= pd
                    pixelNoiseStd = params.PixelNoiseStd + params.PixelNoiseRangeGrowth * range;
                    noisyPixel = truePixel + pixelNoiseStd * randn(2, 1);

                    det = buildDetection(t, i, robot, noisyPixel, pixelNoiseStd, believedPose, ...
                        params, gt.ReportedPoseVariance, false);
                    detections(end + 1) = det; %#ok<AGROW>
                end
            end

            if params.FalsePositivesEnabled && rand() <= params.FalsePositiveRate / robot.FrameRate
                clutterPixel = [robot.Intrinsics.ImageWidth * rand(); robot.Intrinsics.ImageHeight * rand()];
                pixelNoiseStd = params.PixelNoiseStd;

                det = buildDetection(t, i, robot, clutterPixel, pixelNoiseStd, believedPose, ...
                    params, gt.ReportedPoseVariance, true);
                detections(end + 1) = det; %#ok<AGROW>
            end
        end
    end

    [~, order] = sort([detections.t]);
    detections = detections(order);

end

function pose = poseAtIndex(gt, robotIdx, idx, believed)
%POSEATINDEX Pose struct (see COMPUTECAMERAPOSE) for one robot/sample.
%   BELIEVED selects the believed pose (position/yaw/head-pitch biased by
%   the robot's own localisation error, body assumed perfectly upright --
%   see SIMPARAMS) or the true pose (including the real walking wobble).

    r = gt.Robots(robotIdx);

    if believed
        pose.Position  = r.Position(:, idx).' + [r.ErrX(idx), r.ErrY(idx)];
        pose.Yaw       = r.Yaw(idx) + r.ErrYaw(idx);
        pose.Roll      = 0;
        pose.Pitch     = 0;
        pose.HeadYaw   = r.HeadYaw(idx);
        pose.HeadPitch = r.HeadPitch(idx) + r.ErrHeadPitch(idx);
    else
        pose.Position  = r.Position(:, idx).';
        pose.Yaw       = r.Yaw(idx);
        pose.Roll      = r.Roll(idx);
        pose.Pitch     = r.Pitch(idx);
        pose.HeadYaw   = r.HeadYaw(idx);
        pose.HeadPitch = r.HeadPitch(idx);
    end

end

function det = buildDetection(t, robotID, robot, pixel, pixelNoiseStd, believedPose, params, ...
        reportedPoseVariance, isFalsePositive)
%BUILDDETECTION Assemble one detection message and its bearing covariance.

    [rCNn, Rnc] = frames.computeCameraPose(believedPose, params);

    [bearingCam, Jpix] = pixelToBearing(pixel, robot.Intrinsics); % Jpix: 3x2, d(bearingCam)/d(pixel)
    uNn = Rnc * bearingCam;

    Jpixel = Rnc * Jpix;                    % 3x2, d(uNn)/d(pixel)
    SigmaPixel = (pixelNoiseStd^2) * eye(2);

    h = 1e-5;
    Jang = zeros(3, 2);
    for a = 1:2
        poseP = believedPose;
        poseM = believedPose;
        if a == 1
            poseP.Yaw = believedPose.Yaw + h;
            poseM.Yaw = believedPose.Yaw - h;
        else
            poseP.HeadPitch = believedPose.HeadPitch + h;
            poseM.HeadPitch = believedPose.HeadPitch - h;
        end
        [~, RncP] = frames.computeCameraPose(poseP, params);
        [~, RncM] = frames.computeCameraPose(poseM, params);
        Jang(:, a) = (RncP * bearingCam - RncM * bearingCam) / (2*h);
    end
    SigmaAngle = diag(reportedPoseVariance([3 4])); % believed [yaw headPitch] variance

    P_bearing = Jpixel * SigmaPixel * Jpixel.' + Jang * SigmaAngle * Jang.';
    P_bearing = P_bearing + params.BearingRadialVariance * (uNn * uNn.');
    P_bearing = (P_bearing + P_bearing.') / 2; % symmetrize away round-off

    bearingDensity = GaussianInfo.fromMoment(P_bearing);

    det.t               = t;
    det.robotID         = robotID;
    det.pixel            = pixel;
    det.rCNn             = rCNn;
    det.Rnc              = Rnc;
    det.uNn              = uNn;
    det.P_bearing        = bearingDensity.cov();
    det.S_bearing        = bearingDensity.sqrtCov();
    det.isFalsePositive  = isFalsePositive;

end
