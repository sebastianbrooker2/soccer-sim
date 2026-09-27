% Return h(x) and derivatives w.r.t. x
function [h, dhdx, d2hdx2] = predict(obj, x, system)

h = x(1:2) - obj.ObserverPosition;

if nargout >= 2
    %               dh_i
    % dhdx(i, j) = ------
    %               dx_j
    dhdx = [1 0 0 0; ...
            0 1 0 0];
end

if nargout >= 3
    %                     d^2 h_i
    % d2h2dx(i, j, k) = -----------
    %                    dx_j dx_k
    d2hdx2 = zeros(2, 4, 4);   % h is linear in x, so the Hessian is zero
end
