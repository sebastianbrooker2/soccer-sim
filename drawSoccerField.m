function ax = drawSoccerField(cfg)
%DRAWSOCCERFIELD Draw a 2D top-down soccer field.
%   AX = DRAWSOCCERFIELD(CFG) draws the field described by CFG (see
%   FIELDCONFIG) into a new figure and returns the axes handle.
%
%   The origin is the centre of the pitch. X runs goal-to-goal, Y runs
%   touchline-to-touchline.

    if nargin < 1
        cfg = fieldConfig();
    end

    grassColor = [0.16 0.55 0.22];
    lineColor  = [1 1 1];
    lineWidth  = 2;

    L      = cfg.FieldLength;
    W      = cfg.FieldWidth;
    border = cfg.BorderStripMinWidth;

    fig = figure('Name', 'Soccer Simulator', 'Color', grassColor, 'NumberTitle', 'off');
    ax = axes('Parent', fig);
    hold(ax, 'on');
    axis(ax, 'equal');
    axis(ax, [-L/2-border-1, L/2+border+1, -W/2-border-1, W/2+border+1]);
    set(ax, 'Color', grassColor, 'XColor', 'none', 'YColor', 'none');

    % Pitch surface, including the border strip
    patch(ax, [-1 1 1 -1]*(L/2+border), [-1 -1 1 1]*(W/2+border), grassColor, ...
        'EdgeColor', 'none');

    % Outer boundary
    rectangle(ax, 'Position', [-L/2, -W/2, L, W], ...
        'EdgeColor', lineColor, 'LineWidth', lineWidth);

    % Halfway line
    plot(ax, [0 0], [-W/2 W/2], 'Color', lineColor, 'LineWidth', lineWidth);

    % Centre circle and spot
    r = cfg.CenterCircleDiameter / 2;
    th = linspace(0, 2*pi, 100);
    plot(ax, r*cos(th), r*sin(th), 'Color', lineColor, 'LineWidth', lineWidth);
    plot(ax, 0, 0, 'o', 'MarkerFaceColor', lineColor, 'MarkerEdgeColor', lineColor, 'MarkerSize', 4);

    % Left (-x) and right (+x) ends
    drawEnd(ax, cfg, -1, lineColor, lineWidth);
    drawEnd(ax, cfg,  1, lineColor, lineWidth);

    % Left hold on: callers add players/markers on top of the field.

end

function drawEnd(ax, cfg, side, lineColor, lineWidth)
%DRAWEND Draw the penalty area, goal area, penalty mark and goal for one
%   end of the field. SIDE is -1 for the end at x = -FieldLength/2, or
%   +1 for the end at x = +FieldLength/2.

    L = cfg.FieldLength;

    goalLineX = side * L/2;
    inward    = -side; % unit direction pointing into the field

    % Penalty area
    paX = goalLineX + inward * cfg.PenaltyAreaLength;
    rectangle(ax, 'Position', [min(goalLineX, paX), -cfg.PenaltyAreaWidth/2, ...
        abs(paX - goalLineX), cfg.PenaltyAreaWidth], ...
        'EdgeColor', lineColor, 'LineWidth', lineWidth);

    % Goal area
    gaX = goalLineX + inward * cfg.GoalAreaLength;
    rectangle(ax, 'Position', [min(goalLineX, gaX), -cfg.GoalAreaWidth/2, ...
        abs(gaX - goalLineX), cfg.GoalAreaWidth], ...
        'EdgeColor', lineColor, 'LineWidth', lineWidth);

    % Penalty mark
    pmX = goalLineX + inward * cfg.PenaltyMarkDistance;
    plot(ax, pmX, 0, 'o', 'MarkerFaceColor', lineColor, 'MarkerEdgeColor', lineColor, 'MarkerSize', 4);

    % Goal frame, drawn outside the field boundary
    goalX = goalLineX - inward * cfg.GoalDepth;
    rectangle(ax, 'Position', [min(goalLineX, goalX), -cfg.GoalWidth/2, ...
        abs(goalX - goalLineX), cfg.GoalWidth], ...
        'EdgeColor', lineColor, 'LineWidth', lineWidth);

end
