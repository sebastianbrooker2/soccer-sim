function results = runDemo()
%RUNDEMO Smoke test: 2 synthetic matches x 60 s, baseline vs shared SRIF tracker.
results = runEvaluation(struct('nMatches', 2, 'duration', 60, 'runName', 'demo'));
end
