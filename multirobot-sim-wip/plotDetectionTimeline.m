function plotDetectionTimeline(detections, params)
%PLOTDETECTIONTIMELINE Which robot detected the ball, and when.
%   PLOTDETECTIONTIMELINE(DETECTIONS, PARAMS) draws a scatter of
%   DETECTIONS (see GENERATEDETECTIONS) time vs. robot ID, with false
%   positives marked separately.

    figure('Name', 'Detection timeline', 'Color', 'w');
    ax = axes();
    hold(ax, 'on');

    isFP  = [detections.isFalsePositive];
    tAll  = [detections.t];
    idAll = [detections.robotID];

    plot(ax, tAll(~isFP), idAll(~isFP), 'o', 'MarkerFaceColor', [0.1 0.6 0.2], ...
        'MarkerEdgeColor', 'k', 'LineStyle', 'none', 'DisplayName', 'detection');
    if any(isFP)
        plot(ax, tAll(isFP), idAll(isFP), 'x', 'Color', [0.8 0.1 0.1], ...
            'LineWidth', 1.5, 'LineStyle', 'none', 'DisplayName', 'false positive');
    end

    xlabel(ax, 'Time (s)');
    ylabel(ax, 'Robot ID');
    yticks(ax, 1:params.NumRobots);
    ylim(ax, [0.5, params.NumRobots + 0.5]);
    grid(ax, 'on');
    legend(ax, 'Location', 'best');
    title(ax, 'Ball detections over time');

end
