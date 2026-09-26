function h = drawBall(ax, ball)
%DRAWBALL Draw a stationary ball and a hidden detection readout.
%   H = DRAWBALL(AX, BALL) draws BALL (see CREATEBALL) as a small purple
%   circle, plus a detection ring and confidence label that start hidden
%   and are only revealed by UPDATEBALLDETECTION while the ball is
%   detected.

    ballColor = [0.55 0.15 0.75];

    th = linspace(0, 2*pi, 30);
    bx = ball.Position(1) + ball.Radius * cos(th);
    by = ball.Position(2) + ball.Radius * sin(th);
    h.Ball = patch(ax, bx, by, ballColor, 'EdgeColor', [0.1 0.1 0.1], 'LineWidth', 1);

    ringR = ball.Radius * 1.5;
    rx = ball.Position(1) + ringR * cos(th);
    ry = ball.Position(2) + ringR * sin(th);
    h.DetectionRing = plot(ax, rx, ry, 'Color', [0.1 0.9 0.2], 'LineWidth', 2, 'Visible', 'off');

    h.Label = text(ax, ball.Position(1), ball.Position(2) + ringR + 0.3, '', ...
        'Color', [0.1 0.9 0.2], 'FontWeight', 'bold', 'FontSize', 10, ...
        'HorizontalAlignment', 'center', 'Visible', 'off');

end
