function X = simulate(obj, m)

arguments (Input)
    obj (1, 1) GaussianBase
    m (1, 1) uint64 = 1             % Number of samples to generate
end

arguments (Output)
    X (:, :) double                 % n-by-m matrix of samples, where n = obj.dim()
end

% Draw m realisations of a Gaussian random variable
mu = obj.mean();
S  = obj.sqrtCov();

W = randn(obj.dim(), double(m));    % W ~ N(0, I)
X = mu + S.'*W;                     % X = mu + S.'*W ~ N(mu, S.'*S)
