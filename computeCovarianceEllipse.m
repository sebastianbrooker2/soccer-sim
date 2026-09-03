function [ellX, ellY] = computeCovarianceEllipse(player, nSigma)
%COMPUTECOVARIANCEELLIPSE Points on a player's position covariance ellipse.
%   [ELLX, ELLY] = COMPUTECOVARIANCEELLIPSE(PLAYER, NSIGMA) traces the
%   NSIGMA-standard-deviation ellipse of PLAYER.Covariance, centred on
%   PLAYER.Position. NSIGMA defaults to 2.

    if nargin < 2
        nSigma = 2;
    end

    [V, D] = eig(player.Covariance);
    th = linspace(0, 2*pi, 60);
    unitCircle = [cos(th); sin(th)];
    ellipse = V * sqrt(D) * nSigma * unitCircle;

    ellX = player.Position(1) + ellipse(1, :);
    ellY = player.Position(2) + ellipse(2, :);

end
