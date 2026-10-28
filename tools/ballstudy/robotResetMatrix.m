function M = robotResetMatrix(E)
%ROBOTRESETMATRIX Nx3 [t x y] (from-position) for params.tracker.robotResetEvents.
r = E(E.type == "robot", :); M = [r.t r.from_x r.from_y];
end
