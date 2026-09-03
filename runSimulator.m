% RUNSIMULATOR Entry point for the soccer simulator.
%   Run this script to open a 2D visualisation of the field with a
%   player walking a smooth, curving path while its gaze sweeps
%   independently of its walking direction. Close the figure window to
%   stop.

cfg = fieldConfig();
ax = drawSoccerField(cfg);

player = createPlayer([-3, 1], deg2rad(20), [0.3 0.05; 0.05 0.15]);
h = drawPlayer(ax, player);

fig = ancestor(ax, 'figure');
dt = 0.03; % seconds per simulation step

while isvalid(fig)
    player = stepPlayerWalk(player, cfg, dt);
    updatePlayer(h, player);
    drawnow limitrate;
    pause(dt);
end

% test 