function updatePlayer(h, player)
%UPDATEPLAYER Move an already-drawn player's graphics to a new state.
%   UPDATEPLAYER(H, PLAYER) updates the handles H (returned by
%   DRAWPLAYER) in place to match PLAYER's current position, heading and
%   covariance, without recreating any graphics objects.

    [fovX, fovY, arcX, arcY] = computeFOVGeometry(player);
    set(h.FOVPatch, 'XData', fovX, 'YData', fovY);
    set(h.FOVLine1, 'XData', [player.Position(1) arcX(1)],   'YData', [player.Position(2) arcY(1)]);
    set(h.FOVLine2, 'XData', [player.Position(1) arcX(end)], 'YData', [player.Position(2) arcY(end)]);

    [ellX, ellY] = computeCovarianceEllipse(player);
    set(h.Ellipse, 'XData', ellX, 'YData', ellY);

    set(h.Marker, 'XData', player.Position(1), 'YData', player.Position(2));

    set(h.HeadingLine, ...
        'XData', [player.Position(1), player.Position(1) + h.HeadingLength * cos(player.Heading)], ...
        'YData', [player.Position(2), player.Position(2) + h.HeadingLength * sin(player.Heading)]);

end
% test