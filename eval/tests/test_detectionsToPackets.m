function tests = test_detectionsToPackets
tests = functiontests(localfunctions);
end

function [u, v, w, h] = project(cam, ps, hy, hp, X, kind, dims)
% synthetic pinhole projection of a world point (ball centre or robot box)
p = cam.pitch + hp; yaw = cam.yaw + hy + ps.theta;
f = [cos(p); 0; -sin(p)]; r = [0; -1; 0]; dn = [-sin(p); 0; -cos(p)];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
ax = Rz*f; rt = Rz*r; dw = Rz*dn;
Ry = [cos(hp) 0 sin(hp); 0 1 0; -sin(hp) 0 cos(hp)];
Rzh = [cos(hy) -sin(hy) 0; sin(hy) cos(hy) 0; 0 0 1];
Rb = [cos(ps.theta) -sin(ps.theta) 0; sin(ps.theta) cos(ps.theta) 0; 0 0 1];
pb = cam.pivot + Rzh*Ry*cam.offset;
cw = [ps.x; ps.y; 0] + Rb*pb; cw(3) = cam.trunkHeight + pb(3);
pr = @(P) deal(cam.cx + cam.fx*((P-cw).'*rt)/((P-cw).'*ax), ...
               cam.cy + cam.fy*((P-cw).'*dw)/((P-cw).'*ax));
if strcmp(kind, 'ball')
    [u, v] = pr(X);
    Z = (X-cw).'*ax;
    w = cam.fx*2*dims/Z; h = cam.fy*2*dims/Z;
else   % robot: foot X (z=0), height dims, box width 0.3 m
    [uf, vf] = pr(X); [~, vt] = pr(X + [0; 0; dims]);
    Z = (X-cw).'*ax;
    u = uf; h = vf - vt; v = (vf + vt)/2; w = cam.fx*0.3/Z;
end
end

function testRoundTrip(tc)
cam = k1SimCamera(0.55);
poses = struct('t', [0; 10], 'robot', [1; 1], 'x', [0; 0], 'y', [0; 0], ...
    'theta', [0.3; 0.3], 'headYaw', [0.2; 0.2], 'headPitch', [0.35; 0.35]);
ps = struct('x', 0, 'y', 0, 'theta', 0.3);
ballW = [2.0; 0.6];  robW = [2.5; 1.0];
% place targets at known body-relative bearings near the view centre
rows = {};
for pass = 1:2
    if pass == 1
        [u, v, w, h] = project(cam, ps, 0.2, 0.35, [ballW; 0.107], 'ball', 0.107);
        rows(end+1,:) = {1.0, 1, 'Ball', u, v, w, h, 0.9}; %#ok<AGROW>
    else
        [u, v, w, h] = project(cam, ps, 0.2, 0.35, [robW; 0], 'robot', 0.95);
        rows(end+1,:) = {1.0, 1, 'Person', u, v, w, h, 0.8}; %#ok<AGROW>
    end
end
rows(end+1,:) = {2.0, 1, 'none', 0, 0, 0, 0, 1};
T = cell2table(rows, 'VariableNames', {'t','robot','class','u_px','v_px','w_px','h_px','conf'});
assertTrue(tc, all(T.u_px(1:2) > 0 & T.u_px(1:2) < cam.width));
assertTrue(tc, all(T.v_px(1:2) > 0 & T.v_px(1:2) < cam.height));
for method = {'ground', 'size', 'fuse'}
    pk = detectionsToPackets(T, cam, poses, struct('rangeMethod', method{1}, 'keepClipped', true));
    assertEqual(tc, numel(pk), 2);
    assertEqual(tc, numel(pk(1).detections), 2);
    assertEqual(tc, numel(pk(2).detections), 0);
    for d = pk(1).detections
        tgt = ballW; if strcmp(d.Class, 'robot'), tgt = robW; end
        est = pk(1).ObserverPosition + d.y(3)*d.y(1:2);
        tol = 1e-6;                        % exact for ground, and for ball size
        if strcmp(d.Class, 'robot') && ~strcmp(method{1}, 'ground')
            tol = 0.1;                     % tilted-camera height-from-pixels is approximate
        end
        assertLessThan(tc, norm(est - tgt), tol);
        assertEqual(tc, norm(d.y(1:2)), 1, 'AbsTol', 1e-12);
    end
    assertEqual(tc, pk(1).LookHeading, 0.3 + 0.2, 'AbsTol', 1e-12);
    assertEqual(tc, pk(1).FOV, 2*atan(160/cam.fx), 'AbsTol', 1e-12);
    assertTrue(tc, pk(1).RangeNoiseStd > 0);
end
end

function testIgnoresClippedAndLandmarks(tc)
cam = k1SimCamera(0.55);
poses = struct('t', [0; 10], 'robot', [1; 1], 'x', [0; 0], 'y', [0; 0], 'theta', [0; 0]);
T = table([1;1;1], [1;1;1], {'Goalpost';'Person';'Ball'}, [100;100;5], [100;239;100], ...
    [10;40;20], [50;40;20], [1;1;1], 'VariableNames', {'t','robot','class','u_px','v_px','w_px','h_px','conf'});
pk = detectionsToPackets(T, cam, poses, struct('headPitch', 0));
assertEqual(tc, numel(pk), 1);
% goalpost ignored, robot bottom clipped dropped, ball clipped at left edge dropped
assertEqual(tc, numel(pk(1).detections), 0);
end

function testCsvAndTwoRobots(tc)
cam = k1SimCamera(0.55);
poses = struct('t', [0;10;0;10], 'robot', [1;1;2;2], 'x', [0;0;3;3], 'y', [0;0;0;0], ...
    'theta', [0;0;pi;pi]);
ps1 = struct('x', 0, 'y', 0, 'theta', 0); ps2 = struct('x', 3, 'y', 0, 'theta', pi);
B = [1.5; 0.1];
[u1, v1, w1, h1] = project(cam, ps1, 0, 0, [B; 0.107], 'ball', 0.107);
[u2, v2, w2, h2] = project(cam, ps2, 0, 0, [B; 0.107], 'ball', 0.107);
f = [tempname '.csv'];
fid = fopen(f, 'w'); fprintf(fid, 't,robot,class,u_px,v_px,w_px,h_px,conf\n');
fprintf(fid, '1.0,1,ball,%.9f,%.9f,%.9f,%.9f,0.9\n', u1, v1, w1, h1);
fprintf(fid, '1.1,2,ball,%.9f,%.9f,%.9f,%.9f,0.9\n', u2, v2, w2, h2);
fclose(fid); cleanup = onCleanup(@() delete(f)); %#ok<NASGU>
pk = detectionsToPackets(f, cam, poses, struct('rangeMethod', 'ground'));
assertEqual(tc, [pk.RobotID], [1 2]);
for i = 1:2
    d = pk(i).detections(1);
    assertLessThan(tc, norm(pk(i).ObserverPosition + d.y(3)*d.y(1:2) - B), 1e-6);
end
end
