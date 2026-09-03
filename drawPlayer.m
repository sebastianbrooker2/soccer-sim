function h = drawPlayer(ax, player)
%DRAWPLAYER Draw a soccer player on an existing field axes.
%   H = DRAWPLAYER(AX, PLAYER) draws the player's position and heading, a
%   2-sigma covariance ellipse, and a shaded field-of-view cone, and
%   returns a struct of graphics handles for use with UPDATEPLAYER.

    h.PlayerColor = [0.95 0.35 0.05];
    h.FOVColor    = [1.00 0.90 0.10];

    [fovX, fovY, arcX, arcY] = computeFOVGeometry(player);
    h.FOVPatch = patch(ax, fovX, fovY, h.FOVColor, 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    h.FOVLine1 = plot(ax, [player.Position(1) arcX(1)],   [player.Position(2) arcY(1)],   'Color', h.FOVColor, 'LineWidth', 0.75);
    h.FOVLine2 = plot(ax, [player.Position(1) arcX(end)], [player.Position(2) arcY(end)], 'Color', h.FOVColor, 'LineWidth', 0.75);
    h.FOVLine1.Color(4) = 0.4;
    h.FOVLine2.Color(4) = 0.4;

    [ellX, ellY] = computeCovarianceEllipse(player);
    h.Ellipse = plot(ax, ellX, ellY, 'Color', h.PlayerColor, 'LineWidth', 1.5);

    h.Marker = plot(ax, player.Position(1), player.Position(2), 'o', ...
        'MarkerFaceColor', h.PlayerColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);

    h.HeadingLength = 0.6;
    h.HeadingLine = plot(ax, ...
        [player.Position(1), player.Position(1) + h.HeadingLength * cos(player.Heading)], ...
        [player.Position(2), player.Position(2) + h.HeadingLength * sin(player.Heading)], ...
        'Color', h.PlayerColor, 'LineWidth', 2);

end
