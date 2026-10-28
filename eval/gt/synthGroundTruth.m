function gt = synthGroundTruth(duration, dt, seed, csvFile)
%SYNTHGROUNDTRUTH Plausible 3v3 match on a 14x9 m field (deterministic by seed).
%   Ball: rolling friction, kicked toward goals by the nearest robot; robots
%   chase the ball (nearest two per team) or hold a formation spot, <=0.5 m/s.
%   Team 1 attacks +x, team 2 attacks -x. Optional csvFile -> writes CSV.
if nargin < 1, duration = 60; end
if nargin < 2, dt = 0.05; end
if nargin < 3, seed = 1; end
rs = RandStream('mt19937ar','Seed',seed);
L = 14; W = 9; vmax = 0.5; wmax = 2.0; kappa = 0.4; kickR = 0.3;
Nt = floor(duration/dt)+1;
home = [-5 -2.5 -2.5 5 2.5 2.5; 0 2.5 -2.5 0 -2.5 2.5];
home = [-5 -2.5 -2.5 5 2.5 2.5; 0 2.5 -2.5 0 -2.5 2.5];
P = home + 0.3*randn(rs,2,6); th = [zeros(1,3) pi*ones(1,3)];
% kick cooldown per robot to avoid repeated re-kicks
ball = [0;0]; bv = [0;0]; cool = 0;
gt.t = (0:Nt-1).'*dt; gt.ball = zeros(Nt,2); gt.pos = zeros(Nt,2,6);
gt.theta = zeros(Nt,6); gt.team = [1 1 1 2 2 2];
for i = 1:Nt
    gt.ball(i,:) = ball.'; gt.pos(i,:,:) = reshape(P,1,2,6); gt.theta(i,:) = th;
    % robot targets
    tgt = home;
    for tm = 1:2
        idx = (tm-1)*3 + (1:3);
        d = vecnorm(P(:,idx) - ball, 2, 1);
        [~,o] = sort(d);
        chasers = idx(o(1:2));
        tgt(:,chasers(1)) = ball;
        % second chaser supports offset behind ball
        tgt(:,chasers(2)) = ball + [(tm==1)*-1.5 + (tm==2)*1.5; 1.5*sign(P(2,chasers(2))+1e-3)];
        others = setdiff(idx, chasers);
        tgt(:,others) = home(:,others) + [0.4*ball(1);0.3*ball(2)];
    end
    tgt(1,:) = max(min(tgt(1,:), L/2-0.3), -L/2+0.3);
    tgt(2,:) = max(min(tgt(2,:), W/2-0.3), -W/2+0.3);
    % move robots: turn-rate limited heading, speed limited
    for k = 1:6
        dv = tgt(:,k) - P(:,k); dn = norm(dv);
        if dn > 0.05
            des = atan2(dv(2),dv(1));
            e = mod(des - th(k) + pi, 2*pi) - pi;
            th(k) = th(k) + max(min(e, wmax*dt), -wmax*dt);
            sp = min(vmax, dn/dt) * max(0, cos(e));
            P(:,k) = P(:,k) + sp*dt*[cos(th(k)); sin(th(k))];
        else
            th(k) = th(k) + 0.5*dt*randn(rs);
        end
    end
    % ball dynamics
    cool = max(0, cool-dt);
    if cool == 0
        d = vecnorm(P - ball, 2, 1);
        [dm,k] = min(d);
        if dm < kickR
            goal = [7*(2*(k<=3)-1); 0] + [0; 1.0*randn(rs)];
            u = goal - ball; u = u/norm(u);
            u = u + 0.15*randn(rs,2,1); u = u/norm(u);
            bv = (1.5 + 2.0*rand(rs)) * u; cool = 1.0;
        end
    end
    ball = ball + bv*dt; bv = bv*exp(-kappa*dt);
    % walls: goals mouth |y|<1.3 lets ball cross, then reset to centre
    if abs(ball(1)) > L/2
        if abs(ball(2)) < 1.3, ball = [0;0]; bv = [0;0]; cool = 2;
        else, ball(1) = sign(ball(1))*L/2; bv(1) = -0.5*bv(1); end
    end
    if abs(ball(2)) > W/2, ball(2) = sign(ball(2))*W/2; bv(2) = -0.5*bv(2); end
end
if nargin >= 4 && ~isempty(csvFile)
    writeGroundTruthCsv(gt, csvFile);
end
end
