% Paired (robot,t) comparison joint vs own SRIF: where is joint worse? Prints top windows (10 s) per class.
here = fileparts(mfilename('fullpath')); S = load(fullfile(here,'compare3.mat')); W = 10;
for cl = {'ball','opponent'}
    J = S.T.joint(S.T.joint.class==cl{1},:); O = S.T.own(S.T.own.class==cl{1},:);
    assert(isequal(J.t,O.t) && isequal(J.robot,O.robot) && isequal(J.target,O.target));
    okJ = J.hasTrack & J.err<1; okO = O.hasTrack & O.err<1;
    lostBy = okO & ~okJ;            % own good, joint not
    wonBy  = okJ & ~okO;
    wi = floor(J.t/W)*W; u = unique(wi);
    rows = [];
    for w = u'
        m = wi==w; rows(end+1,:) = [w, sum(lostBy(m)), sum(wonBy(m)), sum(m), ...
            mean(J.err(m & J.hasTrack & O.hasTrack & okO)) , mean(O.err(m & J.hasTrack & O.hasTrack & okO))]; %#ok<AGROW>
    end
    T = array2table(rows,'VariableNames',{'t0','own_good_joint_not','joint_good_own_not','n','jointErr_when_both','ownErr_when_both'});
    T.net = T.joint_good_own_not - T.own_good_joint_not; T = sortrows(T,'net');
    fprintf('== %s: total own-good-joint-not %d, joint-good-own-not %d (of %d)\n', cl{1}, sum(lostBy), sum(wonBy), numel(lostBy));
    disp(T(1:8,:));
    % per robot
    for r=1:3, m=J.robot==r; fprintf('  robot %d: lost %d  won %d\n', r, sum(lostBy(m)), sum(wonBy(m))); end
end
