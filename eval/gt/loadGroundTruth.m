function gt = loadGroundTruth(csvFile, dt)
%LOADGROUNDTRUTH Read long-format GT CSV (t,id,team,x,y,theta[,z,roll,pitch,headYaw,headPitch]) onto a uniform grid.
%   gt.t (Nt x1), gt.ball (Nt x2), gt.pos (Nt x2 x6), gt.theta (Nt x6), gt.team (1x6)
%   Optional columns -> gt.z, gt.roll, gt.pitch, gt.headYaw, gt.headPitch (Nt x6, rad; z in m)
%   and gt.hasHead=true. Old CSVs: z=NaN, angles=0, hasHead=false.
if nargin < 2 || isempty(dt), dt = 0.05; end
T = readtable(csvFile);
t0 = min(T.t); t1 = max(T.t);
tg = (t0:dt:t1).';
gt.t = tg;
gt.ball = nan(numel(tg),2);
gt.pos = nan(numel(tg),2,6);
gt.theta = nan(numel(tg),6);
gt.team = [1 1 1 2 2 2];
vn = T.Properties.VariableNames;
gt.hasHead = all(ismember({'z','roll','pitch','headYaw','headPitch'}, vn));
gt.z = nan(numel(tg),6);
gt.roll = zeros(numel(tg),6); gt.pitch = zeros(numel(tg),6);
gt.headYaw = zeros(numel(tg),6); gt.headPitch = zeros(numel(tg),6);
interp = @(tt,v) interp1(tt, v, tg, 'linear', 'extrap');
b = T(T.id==0,:); [tb,ib] = unique(b.t);
gt.ball = [interp(tb,b.x(ib)), interp(tb,b.y(ib))];
for k = 1:6
    r = T(T.id==k,:); [tk,ik] = unique(r.t);
    gt.pos(:,1,k) = interp(tk, r.x(ik));
    gt.pos(:,2,k) = interp(tk, r.y(ik));
    th = unwrap(r.theta(ik));
    gt.theta(:,k) = wrapToPiLocal(interp(tk, th));
    if gt.hasHead
        gt.z(:,k) = interp(tk, r.z(ik));
        % angles are small/continuous but unwrap anyway to avoid +-pi interpolation artefacts
        gt.roll(:,k) = wrapToPiLocal(interp(tk, unwrap(r.roll(ik))));
        gt.pitch(:,k) = wrapToPiLocal(interp(tk, unwrap(r.pitch(ik))));
        gt.headYaw(:,k) = wrapToPiLocal(interp(tk, unwrap(r.headYaw(ik))));
        gt.headPitch(:,k) = wrapToPiLocal(interp(tk, unwrap(r.headPitch(ik))));
    end
    tm = r.team(ik); gt.team(k) = tm(1);
end
end
function a = wrapToPiLocal(a), a = mod(a+pi, 2*pi) - pi; end
