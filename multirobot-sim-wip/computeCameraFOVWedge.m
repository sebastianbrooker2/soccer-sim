function [fovX, fovY, arcX, arcY] = computeCameraFOVWedge(rCNn, Rnc, intrinsics, maxRange)
%COMPUTECAMERAFOVWEDGE Top-down wedge polygon for a camera's horizontal FOV.
%   [FOVX, FOVY, ARCX, ARCY] = COMPUTECAMERAFOVWEDGE(rCNn, Rnc, ...
%   INTRINSICS, MAXRANGE) projects the camera's horizontal field of view
%   (derived from INTRINSICS.fx and INTRINSICS.ImageWidth) onto the
%   ground plane out to MAXRANGE, centred on the camera boresight's yaw
%   (its optical axis projected onto the field's x-y plane). Mirrors the
%   wedge polygon shape produced by COMPUTEFOVGEOMETRY for the existing
%   2D player FOV cone.

    halfFOV = atan2(intrinsics.ImageWidth/2, intrinsics.fx);

    boresightN = Rnc * [0; 0; 1]; % camera optical axis (z-forward) in the field frame
    yaw = atan2(boresightN(2), boresightN(1));

    nArc = 30;
    arcAngles = linspace(yaw - halfFOV, yaw + halfFOV, nArc);
    arcX = rCNn(1) + maxRange * cos(arcAngles);
    arcY = rCNn(2) + maxRange * sin(arcAngles);

    fovX = [rCNn(1), arcX, rCNn(1)];
    fovY = [rCNn(2), arcY, rCNn(2)];

end
