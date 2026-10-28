function copies = runBaselineTrackerTuned(packets, params)
%RUNBASELINETRACKERTUNED runBaselineTracker with params.baseline.mode = 'tuned'.
if nargin < 2, params = struct(); end
if ~isfield(params, 'baseline') || ~isstruct(params.baseline), params.baseline = struct(); end
params.baseline.mode = 'tuned';
copies = runBaselineTracker(packets, params);
end
