function [rCNn, Rnc, Rnb] = computeCameraPose(pose, params)
%COMPUTECAMERAPOSE Field-frame camera position and rotation for a robot pose.
%   [rCNn, Rnc, Rnb] = COMPUTECAMERAPOSE(POSE, PARAMS) chains
%   field (n) -> body (b) -> head (h) -> camera (c). POSE is a struct with
%   fields (all rad / m):
%     Position            [x y] in the field frame
%     Yaw, Roll, Pitch    body orientation (Roll/Pitch = walking wobble)
%     HeadYaw, HeadPitch  head joint angles relative to the body
%   PARAMS supplies HeadHeight, CameraOffsetHead and CameraTiltDown (see
%   SIMPARAMS).
%
%   Rnc maps a camera-frame vector to the field frame: v_n = Rnc*v_c.
%   The camera frame uses the OpenCV convention (z forward, x right, y
%   down); RHC0 below is the fixed rotation from the head frame (x
%   forward, y left, z up -- the same convention as the body/field frame)
%   into that camera convention, i.e. it is not a tunable parameter.

    Rnb = frames.rotz(pose.Yaw) * frames.roty(pose.Pitch) * frames.rotx(pose.Roll);
    Rbh = frames.rotz(pose.HeadYaw) * frames.roty(pose.HeadPitch);

    Rhc0  = [0 0 1; -1 0 0; 0 -1 0];                % camera (x-right,y-down,z-fwd) -> head (x-fwd,y-left,z-up)
    Rtilt = frames.rotx(-params.CameraTiltDown);   % small fixed downward tilt, about the camera's own x (right) axis
    Rhc   = Rhc0 * Rtilt;

    Rnc = Rnb * Rbh * Rhc;

    rBNn = [pose.Position(:); 0];
    rHBb = [0; 0; params.HeadHeight];
    rCHh = params.CameraOffsetHead(:);

    rCNn = rBNn + Rnb * rHBb + Rnb * Rbh * rCHh;

end
