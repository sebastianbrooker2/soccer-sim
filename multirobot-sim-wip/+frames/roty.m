function R = roty(theta)
%ROTY Rotation matrix for a right-handed rotation of THETA radians about y.
%   R = ROTY(THETA) is such that v_rotated = R*v.

    c = cos(theta);
    s = sin(theta);
    R = [c 0 s; 0 1 0; -s 0 c];

end
