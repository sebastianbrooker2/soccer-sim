function [pixel, J] = projectPoint(pC, intrinsics)
%PROJECTPOINT Pinhole projection of a camera-frame point, with optional
%   radial distortion.
%   [PIXEL, J] = PROJECTPOINT(PC, INTRINSICS) projects PC (3x1, an
%   OpenCV camera-frame point with PC(3) > 0 in front of the camera) to
%   pixel coordinates [u;v] using INTRINSICS.fx/fy/cx/cy and, if
%   INTRINSICS.DistortionCoeffs = [k1 k2] is non-zero, a radial
%   distortion model applied in normalized coordinates. J is the 2x3
%   Jacobian d(pixel)/d(pC).

    pC = pC(:);
    x = pC(1); y = pC(2); z = pC(3);

    xn = x/z;
    yn = y/z;

    k1 = intrinsics.DistortionCoeffs(1);
    k2 = intrinsics.DistortionCoeffs(2);
    r2 = xn^2 + yn^2;
    distFactor = 1 + k1*r2 + k2*r2^2;

    xd = xn*distFactor;
    yd = yn*distFactor;

    pixel = [intrinsics.fx*xd + intrinsics.cx; intrinsics.fy*yd + intrinsics.cy];

    if nargout >= 2
        dxn = [1/z, 0, -x/z^2];
        dyn = [0, 1/z, -y/z^2];
        dr2 = 2*xn*dxn + 2*yn*dyn;
        ddist = (k1 + 2*k2*r2) * dr2;

        dxd = distFactor*dxn + xn*ddist;
        dyd = distFactor*dyn + yn*ddist;

        J = [intrinsics.fx*dxd; intrinsics.fy*dyd];
    end

end
