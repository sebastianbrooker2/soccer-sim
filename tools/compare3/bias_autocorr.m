% Empirical autocorrelation of the believed-minus-true observer position (AR(1) bias) from the packets.
if ~exist('COP','var'), COP = fullfile(tempdir,'cop.mat'); end
Z = load(COP, 'packets', 'gt'); P = Z.packets; gt = Z.gt;
fprintf('configured: std %.3f m/axis, tau %.1f s\n', sqrt(P(1).ObserverCovariance(1,1)), 5);
for r = 1:3
    pr = P([P.RobotID] == r); t = [pr.time]; b = zeros(numel(pr), 2);
    for i = 1:numel(pr), k = min(max(round((t(i)-gt.t(1))/(gt.t(2)-gt.t(1)))+1,1),numel(gt.t)); b(i,:) = (pr(i).ObserverPosition(:) - squeeze(gt.pos(k,:,r)).').'; end
    fprintf('robot %d: n=%d std (%.3f,%.3f) ', r, numel(t), std(b(:,1)), std(b(:,2)));
    for lag = [0.5 1 2 5 10]
        j = interp1(t, 1:numel(t), t + lag, 'next'); ok = ~isnan(j); ii = find(ok); jj = j(ok); c = corr(b(ii,1), b(jj,1));
        fprintf(' rho(%gs)=%.2f', lag, c);
    end
    fprintf('\n');
end
