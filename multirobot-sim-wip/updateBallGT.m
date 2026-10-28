function updateBallGT(h, ballPositionXY, radius)
%UPDATEBALLGT Move an already-drawn ball's graphics (see DRAWBALLGT) to a new position.

    th = linspace(0, 2*pi, 30);
    bx = ballPositionXY(1) + radius * cos(th);
    by = ballPositionXY(2) + radius * sin(th);
    set(h.Ball, 'XData', bx, 'YData', by);

end
