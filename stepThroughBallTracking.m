function stepThroughBallTracking(event_queue, robots)
%STEPTHROUGHBALLTRACKING Interactive keyboard step-through of one run.
%   STEPTHROUGHBALLTRACKING(EVENT_QUEUE, ROBOTS) replays an already-run
%   EVENT_QUEUE (e.g. from RUN_TRACK_BALL_STATIONARY_ROBOTS) one event at
%   a time -- either a pure PREDICT time update, or a
%   MEASUREMENTBALLBEARING update -- so you can see exactly what each
%   one does to the estimate. Every event's resulting SYSTEM state was
%   already saved during the run (EVENT.saveSystemState), so this only
%   scrubs through what's already there; nothing is recomputed live.
%
%   ROBOTS(r) can be either STATIC (fields Position/Heading/Covariance/
%   FOV/Range, as in RUN_TRACK_BALL_STATIONARY_ROBOTS) or MOVING (also
%   has a time vector ROBOTS(r).t and per-sample history fields
%   TruePosition/TrueHeading/BelievedPosition/BelievedHeading/
%   BelievedCov -- see RUN_MOVING_ROBOTS_STATIONARY_BALL), in which case
%   the robot's marker/cone/ellipse are moved each step to whichever
%   sample is nearest the current event's time, and its true position is
%   also shown as a small dim marker for reference.
%
%   Keys: right arrow / space = next event, left arrow = previous event,
%   home/end = jump to first/last, q/escape = close.

    N = numel(event_queue);
    numRobots = numel(robots);

    ax = drawSoccerField(fieldConfig());
    fig = ancestor(ax, 'figure');
    set(fig, 'KeyPressFcn', @onKeyPress, 'Position', [100 100 1000 620]);
    set(ax, 'Units', 'normalized', 'Position', [0.03 0.05 0.64 0.90]); % leave the right ~30% for the info panel

    robotIsMoving = false(1, numRobots);
    hRobotPlayer  = cell(1, numRobots);
    hRobotTrue    = gobjects(1, numRobots);
    hRobotLabel   = gobjects(1, numRobots);

    for r = 1:numRobots
        robotIsMoving(r) = isfield(robots(r), 't');
        if robotIsMoving(r)
            player = believedPlayerAtTime(robots(r), robots(r).t(1));
        else
            player.Position   = robots(r).Position.';
            player.Heading    = robots(r).Heading;
            player.Covariance = robots(r).Covariance;
            player.FOV        = robots(r).FOV;
            player.Range      = robots(r).Range;
        end
        hRobotPlayer{r} = drawPlayer(ax, player);
        hRobotLabel(r) = text(ax, player.Position(1), player.Position(2) - 0.6, sprintf('R%d', r), ...
            'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
        if robotIsMoving(r)
            hRobotTrue(r) = plot(ax, nan, nan, 'x', 'Color', [0.65 0.65 0.65], ...
                'MarkerSize', 8, 'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end

    hTrue       = plot(ax, nan, nan, 'o', 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
    hEstMarker  = plot(ax, nan, nan, '^', 'MarkerFaceColor', [0.1 0.7 0.2], 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
    hEstEllipse = plot(ax, nan, nan, 'Color', [0.1 0.7 0.2], 'LineWidth', 1.5);
    hMeasLine   = plot(ax, nan, nan, '--', 'Color', [1 0.85 0.1], 'LineWidth', 1.5);
    legend(ax, [hTrue, hEstMarker, hEstEllipse, hMeasLine], {'true ball', 'estimate', '2\sigma', 'fused measurement'}, ...
        'TextColor', 'w', 'Location', 'best');

    panelBg = [0.12 0.12 0.12];
    hInfoPanel = uipanel(fig, 'Units', 'normalized', 'Position', [0.70 0.05 0.28 0.90], ...
        'BackgroundColor', panelBg, 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 10, ...
        'Title', 'Event');
    hInfoText = uicontrol(hInfoPanel, 'Style', 'text', 'Units', 'normalized', 'Position', [0.05 0.04 0.90 0.93], ...
        'BackgroundColor', panelBg, 'ForegroundColor', 'w', 'HorizontalAlignment', 'left', ...
        'FontName', 'FixedWidth', 'FontSize', 9, 'String', '');

    render(1);

    function onKeyPress(~, evt)
        k = currentIndex();
        switch evt.Key
            case {'rightarrow', 'space'}
                k = min(k + 1, N);
            case 'leftarrow'
                k = max(k - 1, 1);
            case 'home'
                k = 1;
            case 'end'
                k = N;
            case {'q', 'escape'}
                close(fig);
                return
            otherwise
                return
        end
        render(k);
    end

    function idx = currentIndex()
        idx = fig.UserData;
        if isempty(idx)
            idx = 1;
        end
    end

    function render(idx)
        fig.UserData = idx;

        ev  = event_queue(idx);
        sys = ev.system;

        for r = 1:numRobots
            if ~robotIsMoving(r)
                continue
            end
            player = believedPlayerAtTime(robots(r), ev.time);
            updatePlayer(hRobotPlayer{r}, player);
            set(hRobotLabel(r), 'Position', [player.Position(1), player.Position(2) - 0.6]);
            truePos = truePositionAtTime(robots(r), ev.time);
            set(hRobotTrue(r), 'XData', truePos(1), 'YData', truePos(2));
        end

        set(hTrue, 'XData', sys.x_sim(1), 'YData', sys.x_sim(2));

        mu = sys.density.mean();
        P  = sys.density.cov();
        set(hEstMarker, 'XData', mu(1), 'YData', mu(2));

        est.Position   = mu(1:2).';
        est.Covariance = P(1:2, 1:2);
        [ellX, ellY] = computeCovarianceEllipse(est, 2);
        set(hEstEllipse, 'XData', ellX, 'YData', ellY);

        if isa(ev, 'MeasurementBallBearing')
            d = ev.LastDetection;
            if d.Detected
                % Only draw this when the robot actually detected the ball --
                % that's the only case UPDATE.M fuses anything into density.
                set(hMeasLine, 'XData', [ev.ObserverPosition(1), sys.x_sim(1)], ...
                               'YData', [ev.ObserverPosition(2), sys.x_sim(2)]);
                titleStr   = sprintf('t=%.3fs   MEASUREMENT UPDATE -- Robot %d, FUSED', ev.time, ev.RobotID);
                titleColor = [1 0.85 0.1];
            else
                set(hMeasLine, 'XData', nan, 'YData', nan);
                titleStr   = sprintf('t=%.3fs   Robot %d looked, NOT detected -- PROCESS UPDATE only', ev.time, ev.RobotID);
                titleColor = [0.75 0.75 0.75];
            end
            eventLines = { ...
                sprintf('MEASUREMENT (Robot %d)', ev.RobotID), ...
                sprintf('  detected   %d', d.Detected), ...
                sprintf('  confidence %.2f', d.Confidence), ...
                sprintf('  distance   %.2f m', d.Distance), ...
                sprintf('  angle err  %.1f deg', rad2deg(d.AngleError))};
        else
            set(hMeasLine, 'XData', nan, 'YData', nan);
            titleStr   = sprintf('t=%.3fs   PROCESS UPDATE (predict only, no robot scheduled)', ev.time);
            titleColor = [1 1 1];
            eventLines = {'PREDICT', '  (time update only,', '   no measurement)'};
        end

        sigma = realsqrt(diag(P));
        hInfoPanel.Title = sprintf('Event %d / %d   t=%.3fs', idx, N, ev.time);
        infoLines = [eventLines, {'', ...
            'TRUE', ...
            sprintf('  pos  %7.3f, %7.3f', sys.x_sim(1), sys.x_sim(2)), ...
            sprintf('  vel  %7.3f, %7.3f', sys.x_sim(3), sys.x_sim(4)), ...
            '', ...
            'ESTIMATE', ...
            sprintf('  pos  %7.3f, %7.3f', mu(1), mu(2)), ...
            sprintf('  vel  %7.3f, %7.3f', mu(3), mu(4)), ...
            '', ...
            'SIGMA', ...
            sprintf('  pos  %7.3f, %7.3f', sigma(1), sigma(2)), ...
            sprintf('  vel  %7.3f, %7.3f', sigma(3), sigma(4)), ...
            '', ...
            '<- prev     next ->', ...
            'home/end    q=close'}];
        set(hInfoText, 'String', infoLines);

        title(ax, titleStr, 'Color', titleColor, 'FontWeight', 'bold');
    end
end

function player = believedPlayerAtTime(r, t)
%BELIEVEDPLAYERATTIME PLAYER-shaped struct (see DRAWPLAYER) for a moving
%   robot's believed pose nearest time T.
[~, kk] = min(abs(r.t - t));
player.Position   = r.BelievedPosition(:, kk).';
player.Heading     = r.BelievedHeading(kk);
player.Covariance = diag(r.BelievedCov(1:2, kk));
player.FOV         = r.FOV;
player.Range       = r.Range;
end

function pos = truePositionAtTime(r, t)
%TRUEPOSITIONATTIME True [x y] position of a moving robot nearest time T.
[~, kk] = min(abs(r.t - t));
pos = r.TruePosition(:, kk).';
end
