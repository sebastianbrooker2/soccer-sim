function h = drawBallGT(ax, ballPositionXY, radius)
%DRAWBALLGT Draw the ball's true ground-truth position as a filled circle.
%   H = DRAWBALLGT(AX, BALLPOSITIONXY, RADIUS) draws the top-down [x y]
%   ball position. Distinct from DRAWBALL/UPDATEBALLDETECTION, which
%   pair a stationary ball with a single player's detection-confidence
%   readout; this one just tracks the ball's true, moving position for
%   the multi-robot ground-truth animation.

    ballColor = [0.55 0.15 0.75];

    th = linspace(0, 2*pi, 30);
    bx = ballPositionXY(1) + radius * cos(th);
    by = ballPositionXY(2) + radius * sin(th);
    h.Ball = patch(ax, bx, by, ballColor, 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1);

end
