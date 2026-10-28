function R = rotx(theta)
%ROTX Rotation matrix for a right-handed rotation of THETA radians about x.
%   R = ROTX(THETA) is such that v_rotated = R*v.

    c = cos(theta);
    s = sin(theta);
    R = [1 0 0; 0 c -s; 0 s c];

end
