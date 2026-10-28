function [camPos, Rc] = cameraPoseK1(cam, trunkXYZ, theta, roll, pitch, headYaw, headPitch)
%CAMERAPOSEK1 World pose of the K1 head camera.
%   Trunk orientation R = Rz(theta)*Ry(pitch)*Rx(roll) (pitch + = nose DOWN,
%   same sign convention as headPitch); head = Rz(headYaw)*Ry(headPitch)
%   (pitch + = down). Camera position = trunk + R*(cam.pivot + Rh*cam.offset).
%   Rc = R*Rh: columns are the camera optical axis (forward), left and up
%   vectors in the world frame (camera looks along +x of this frame).
Rz = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
Ry = @(a) [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
Rx = @(a) [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
R  = Rz(theta) * Ry(pitch) * Rx(roll);
Rh = Rz(headYaw) * Ry(headPitch);
camPos = trunkXYZ(:) + R * (cam.pivot(:) + Rh * cam.offset(:));
Rc = R * Rh;
end
