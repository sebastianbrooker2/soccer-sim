function R = rotz(theta)
%ROTZ Rotation matrix for a right-handed rotation of THETA radians about z.
%   R = ROTZ(THETA) is such that v_rotated = R*v.

    c = cos(theta);
    s = sin(theta);
    R = [c -s 0; s c 0; 0 0 1];

end
