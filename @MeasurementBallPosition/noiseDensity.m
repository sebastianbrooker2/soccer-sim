function out = noiseDensity(obj, system)
%NOISEDENSITY Combined measurement noise covariance.
%   The observer's own position uncertainty (OBSERVERCOVARIANCE)
%   translates directly into the relative-position measurement, in
%   addition to the detector's own noise (DETECTIONNOISECOVARIANCE);
%   since the two are independent, their covariances add.

R = obj.DetectionNoiseCovariance + obj.ObserverCovariance;
out = GaussianInfo.fromMoment(R);
