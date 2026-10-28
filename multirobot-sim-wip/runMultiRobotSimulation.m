% RUNMULTIROBOTSIMULATION Entry point for the multi-robot ball-detection
%   simulator. Runs the ground-truth simulation (SIMULATEGROUNDTRUTH),
%   generates per-camera detections (GENERATEDETECTIONS), saves both to
%   disk for later use by a fusion filter, then shows a top-down field
%   animation (ANIMATEMULTIROBOTSIM) and a detection timeline
%   (PLOTDETECTIONTIMELINE).
%
%   Distinct from the existing RUNSIMULATOR.M, which demonstrates a
%   single 2D player and a detection-confidence model rather than a
%   bearing-measurement sensor model.

params = simParams();

gt         = simulateGroundTruth(params);
detections = generateDetections(gt, params);

save('groundTruth.mat', 'gt');
save('detections.mat', 'detections');

fprintf('Simulated %d detections (%d false positives) over %.1f s with %d robots.\n', ...
    numel(detections), sum([detections.isFalsePositive]), params.Duration, params.NumRobots);

animateMultiRobotSim(gt, detections, params);
plotDetectionTimeline(detections, params);
