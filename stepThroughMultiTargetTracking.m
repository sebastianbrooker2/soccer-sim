function stepThroughMultiTargetTracking(mergedEvents, targetQueues, targetNames, initialSystems, robots)
%STEPTHROUGHMULTITARGETTRACKING Interactive keyboard step-through across
%   several independently-tracked targets sharing one measurement event
%   timeline.
%   STEPTHROUGHMULTITARGETTRACKING(MERGEDEVENTS, TARGETQUEUES,
%   TARGETNAMES, INITIALSYSTEMS, ROBOTS) is the multi-target
%   generalisation of STEPTHROUGHBALLTRACKING. Each target (e.g. the
%   ball, or an opponent robot) is tracked by its OWN independent
%   SystemBall-style filter -- three separate instances, not one joint
%   state, since nothing here actually couples them -- so:
%     MERGEDEVENTS   every target's own event queue concatenated and
%                    time-sorted, for one chronological step-through.
%                    Each event still only updates its OWN target's
%                    system; EV.TargetName says which.
%     TARGETQUEUES   cell array, TARGETQUEUES{n} is target N's own
%                    time-sorted event queue (a subset of MERGEDEVENTS).
%     TARGETNAMES    cell array of each target's display name, same
%                    order as TARGETQUEUES.
%     INITIALSYSTEMS cell array of each target's pre-loop SYSTEM, used
%                    for "as of time 0" display before its first event.
%     ROBOTS         the (static) sensing robots, drawn once.
%
%   For the target whose event is currently selected, its true/estimate
%   markers come directly from that event's own saved SYSTEM. For every
%   OTHER target, its markers come from the most recent (<=) event in
%   its OWN queue relative to the current time -- i.e. "whatever we last
%   knew about it".
%
%   Keys: right arrow / space = next event, left arrow = previous event,
%   home/end = jump to first/last, q/escape = close.

    N = numel(mergedEvents);
    numTargets = numel(targetQueues);
    targetColors = lines(numTargets);

    ax = drawSoccerField(fieldConfig());
    fig = ancestor(ax, 'figure');
    set(fig, 'KeyPressFcn', @onKeyPress, 'Position', [100 100 1000 620]);
    set(ax, 'Units', 'normalized', 'Position', [0.03 0.05 0.64 0.90]);

    numRobots = numel(robots);
    robotPans = isfield(robots, 'GazeAmplitude'); % head-pan sweep, if these robots have one (see RUN_5V5_STATIONARY_ROBOTS)
    hRobotPlayer = cell(1, numRobots);

    for r = 1:numRobots
        player.Position   = robots(r).Position.';
        player.Heading    = robots(r).Heading;
        player.Covariance = robots(r).Covariance;
        player.FOV        = robots(r).FOV;
        player.Range      = robots(r).Range;
        if robotPans
            player.GazeOffset = robots(r).GazeAmplitude * sin(robots(r).GazePhase0);
        end
        hRobotPlayer{r} = drawPlayer(ax, player);
        text(ax, robots(r).Position(1), robots(r).Position(2) - 0.6, sprintf('R%d', r), ...
            'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
    end

    hTrue       = gobjects(1, numTargets);
    hEstMarker  = gobjects(1, numTargets);
    hEstEllipse = gobjects(1, numTargets);
    legendHandles = gobjects(1, 2*numTargets);
    legendLabels  = cell(1, 2*numTargets);
    for n = 1:numTargets
        c = targetColors(n, :);
        hTrue(n)       = plot(ax, nan, nan, 'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 9);
        hEstMarker(n)  = plot(ax, nan, nan, '^', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', c, 'LineWidth', 1.5, 'MarkerSize', 8);
        hEstEllipse(n) = plot(ax, nan, nan, '--', 'Color', c, 'LineWidth', 1.5);
        legendHandles(2*n - 1) = hTrue(n);
        legendLabels{2*n - 1}  = sprintf('%s true', targetNames{n});
        legendHandles(2*n)     = hEstEllipse(n);
        legendLabels{2*n}      = sprintf('%s estimate/2\\sigma', targetNames{n});
    end
    hMeasLine = plot(ax, nan, nan, '--', 'Color', [1 0.85 0.1], 'LineWidth', 1.5);
    legend(ax, [legendHandles, hMeasLine], [legendLabels, {'fused measurement'}], ...
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

    function sys = systemAsOf(n, t)
        q = targetQueues{n};
        j = find([q.time] <= t, 1, 'last');
        if isempty(j)
            sys = initialSystems{n};
        else
            sys = q(j).system;
        end
    end

    function render(idx)
        fig.UserData = idx;
        ev = mergedEvents(idx);
        t  = ev.time;

        if robotPans
            for r = 1:numRobots
                player.Position   = robots(r).Position.';
                player.Heading    = robots(r).Heading;
                player.Covariance = robots(r).Covariance;
                player.FOV        = robots(r).FOV;
                player.Range      = robots(r).Range;
                player.GazeOffset = robots(r).GazeAmplitude * sin(robots(r).GazePhase0 + robots(r).GazeAngularSpeed*t);
                updatePlayer(hRobotPlayer{r}, player);
            end
        end

        activeIdx = find(strcmp(targetNames, ev.TargetName), 1);

        for n = 1:numTargets
            if n == activeIdx
                sys = ev.system; % exact, no as-of lookup needed for the event actually selected
            else
                sys = systemAsOf(n, t);
            end

            set(hTrue(n), 'XData', sys.x_sim(1), 'YData', sys.x_sim(2));
            mu = sys.density.mean();
            P  = sys.density.cov();
            set(hEstMarker(n), 'XData', mu(1), 'YData', mu(2));
            est.Position   = mu(1:2).';
            est.Covariance = P(1:2, 1:2);
            [ellX, ellY] = computeCovarianceEllipse(est, 2);
            set(hEstEllipse(n), 'XData', ellX, 'YData', ellY);

            if n == activeIdx
                sysActive = sys; muActive = mu; PActive = P;
            end
        end

        d = ev.LastDetection;
        if d.Detected
            % Only draw this when the robot actually detected the target --
            % that's the only case UPDATE.M fuses anything into density.
            set(hMeasLine, 'XData', [ev.ObserverPosition(1), sysActive.x_sim(1)], ...
                           'YData', [ev.ObserverPosition(2), sysActive.x_sim(2)]);
            titleStr   = sprintf('t=%.3fs   MEASUREMENT UPDATE -- Robot %d -> %s, FUSED', t, ev.RobotID, ev.TargetName);
            titleColor = [1 0.85 0.1];
        else
            set(hMeasLine, 'XData', nan, 'YData', nan);
            titleStr   = sprintf('t=%.3fs   Robot %d looked for %s, NOT detected -- PREDICT only', t, ev.RobotID, ev.TargetName);
            titleColor = [0.75 0.75 0.75];
        end

        sigmaActive = realsqrt(diag(PActive));
        infoLines = { ...
            sprintf('TARGET: %s', ev.TargetName), ...
            sprintf('MEASUREMENT (Robot %d)', ev.RobotID), ...
            sprintf('  detected   %d', d.Detected), ...
            sprintf('  confidence %.2f', d.Confidence), ...
            sprintf('  distance   %.2f m', d.Distance), ...
            sprintf('  angle err  %.1f deg', rad2deg(d.AngleError)), ...
            '', ...
            'TRUE', ...
            sprintf('  pos  %7.3f, %7.3f', sysActive.x_sim(1), sysActive.x_sim(2)), ...
            sprintf('  vel  %7.3f, %7.3f', sysActive.x_sim(3), sysActive.x_sim(4)), ...
            '', ...
            'ESTIMATE', ...
            sprintf('  pos  %7.3f, %7.3f', muActive(1), muActive(2)), ...
            sprintf('  vel  %7.3f, %7.3f', muActive(3), muActive(4)), ...
            '', ...
            'SIGMA', ...
            sprintf('  pos  %7.3f, %7.3f', sigmaActive(1), sigmaActive(2)), ...
            sprintf('  vel  %7.3f, %7.3f', sigmaActive(3), sigmaActive(4)), ...
            '', ...
            '<- prev     next ->', ...
            'home/end    q=close'};

        hInfoPanel.Title = sprintf('Event %d / %d   t=%.3fs', idx, N, t);
        set(hInfoText, 'String', infoLines);
        title(ax, titleStr, 'Color', titleColor, 'FontWeight', 'bold');
    end
end
