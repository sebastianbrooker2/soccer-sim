function cam = k1SimCamera(trunkHeight)
%K1SIMCAMERA Intrinsics/mount of the Booster Studio sim K1 head camera.
%   From ~/.soccer-train/sim/mjcf/K1_22dof.xml: <camera pos="0.0613 0 0.108"
%   fovy="58" resolution="320 240"> on body Head_2 (child of Head_1 at
%   (0.0056,0,0.2149) and Head_2 at +0.033 z). Ideal pinhole, fx=fy,
%   principal point at the image centre, no distortion (camera_manager.py).
%   TRUNKHEIGHT [m]: height of the Trunk origin above ground when standing
%   (NOT in the MJCF; spawn is 0.7 m; measure from cam_xpos and pass it).
if nargin < 1, trunkHeight = 0.55; end
W = 320; H = 240; fovy = deg2rad(58);
fy = H/(2*tan(fovy/2));
cam = struct('fx', fy, 'fy', fy, 'cx', W/2, 'cy', H/2, 'width', W, 'height', H, ...
    'pivot', [0.0056; 0; 0.2149 + 0.033], 'offset', [0.0613; 0; 0.108], ...
    'pitch', 0, 'yaw', 0, 'trunkHeight', trunkHeight);
end
