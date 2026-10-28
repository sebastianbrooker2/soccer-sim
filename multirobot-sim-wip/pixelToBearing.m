function [bearingCamera, J] = pixelToBearing(pixel, intrinsics)
%PIXELTOBEARING Undistorted pixel to unit bearing vector in the camera frame.
%   [BEARINGCAMERA, J] = PIXELTOBEARING(PIXEL, INTRINSICS) inverts the
%   pinhole (+ optional radial distortion) model in PROJECTPOINT: PIXEL
%   is undistorted to normalized coordinates (by fixed-point iteration
%   when INTRINSICS.DistortionCoeffs is non-zero), embedded as
%   [xn; yn; 1] and normalized to a unit vector in the OpenCV camera
%   frame (z forward). J is the 3x2 Jacobian d(bearingCamera)/d(pixel),
%   evaluated by central differences (cheap here, and avoids having to
%   differentiate through the distortion fixed-point iteration by hand).

    bearingCamera = unproject(pixel, intrinsics);

    if nargout >= 2
        h = 1e-4;
        J = zeros(3, 2);
        for i = 1:2
            dp = zeros(2, 1);
            dp(i) = h;
            bPlus  = unproject(pixel + dp, intrinsics);
            bMinus = unproject(pixel - dp, intrinsics);
            J(:, i) = (bPlus - bMinus) / (2*h);
        end
    end

end

function bearingCamera = unproject(pixel, intrinsics)

    xd = (pixel(1) - intrinsics.cx) / intrinsics.fx;
    yd = (pixel(2) - intrinsics.cy) / intrinsics.fy;

    k1 = intrinsics.DistortionCoeffs(1);
    k2 = intrinsics.DistortionCoeffs(2);

    xn = xd;
    yn = yd;
    if k1 ~= 0 || k2 ~= 0
        for iter = 1:8
            r2 = xn^2 + yn^2;
            factor = 1 + k1*r2 + k2*r2^2;
            xn = xd / factor;
            yn = yd / factor;
        end
    end

    v = [xn; yn; 1];
    bearingCamera = v / norm(v);

end
