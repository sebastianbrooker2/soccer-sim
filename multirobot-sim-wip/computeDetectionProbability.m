function pd = computeDetectionProbability(range, maxRange, params)
%COMPUTEDETECTIONPROBABILITY Detection probability p_d, falling off with range.
%   PD = COMPUTEDETECTIONPROBABILITY(RANGE, MAXRANGE, PARAMS) is
%   PARAMS.DetectionProbMax at RANGE = 0, decaying as a Gaussian in
%   RANGE/MAXRANGE with rate PARAMS.DetectionProbRangeFalloff, and
%   clipped to zero beyond MAXRANGE.

    pd = params.DetectionProbMax * exp(-params.DetectionProbRangeFalloff * (range/maxRange).^2);
    pd(range > maxRange) = 0;

end
