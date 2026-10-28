function checkGroundTruthHead(csvFile)
%CHECKGROUNDTRUTHHEAD Load a GT CSV and print head/trunk stats (sanity check).
if nargin < 1, csvFile = fullfile(fileparts(mfilename('fullpath')), 'real01.csv'); end
gt = loadGroundTruth(csvFile, 0.05);
assert(isequal(size(gt.headYaw), [numel(gt.t) 6]) && isequal(size(gt.z), [numel(gt.t) 6]));
fprintf('%s hasHead=%d\n', csvFile, gt.hasHead);
if ~gt.hasHead, disp('old-format CSV: no head data'); return; end
fprintf('headYaw   [%.3f, %.3f] rad\n', min(gt.headYaw(:)), max(gt.headYaw(:)));
fprintf('headPitch [%.3f, %.3f] rad\n', min(gt.headPitch(:)), max(gt.headPitch(:)));
fprintf('trunk z mean %.3f (min %.3f max %.3f)\n', mean(gt.z(:)), min(gt.z(:)), max(gt.z(:)));
fprintf('trunk roll [%.3f %.3f] pitch [%.3f %.3f]\n', min(gt.roll(:)), max(gt.roll(:)), min(gt.pitch(:)), max(gt.pitch(:)));
assert(all(isfinite(gt.z(:))) && max(abs(gt.headYaw(:))) <= 1.0001 && mean(gt.z(:)) > 0.3 && mean(gt.z(:)) < 0.7);
disp('OK');
end
