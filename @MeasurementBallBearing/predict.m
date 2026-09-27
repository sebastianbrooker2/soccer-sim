% Return h(x) and derivatives w.r.t. x
function [h, dhdx, d2hdx2] = predict(obj, x, system)

d = x(1:2) - obj.ObserverPosition; % [dx; dy], observer -> ball
r = norm(d);
u = d / r;

h = u; % 2x1 unit bearing vector -- no periodicity, unlike atan2(dy,dx)

if nargout >= 2
    %               dh_i
    % dhdx(i, j) = ------
    %               dx_j
    % Standard tangent-plane projection for a unit vector: (I - u*u.')/r
    A = (eye(2) - u*u.') / r;
    dhdx = [A, zeros(2, 2)];
end

if nargout >= 3
    %                     d^2 h_i
    % d2h2dx(i, j, k) = -----------
    %                    dx_j dx_k
    % d^2u_i/dd_j dd_k = -(A(i,k)*u(j) + u(i)*A(j,k) + A(i,j)*u(k)) / r
    d2hdx2 = zeros(2, 4, 4);
    for i = 1:2
        for j = 1:2
            for k = 1:2
                d2hdx2(i, j, k) = -(A(i, k)*u(j) + u(i)*A(j, k) + A(i, j)*u(k)) / r;
            end
        end
    end
end
