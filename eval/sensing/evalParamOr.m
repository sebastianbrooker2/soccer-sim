function v = evalParamOr(params, names, default)
%EVALPARAMOR Look up a parameter in PARAMS under any of NAMES (char or cell),
%   also searching one level into sub-structs (e.g. params.sensor.fov);
%   DEFAULT if absent/empty. Lets sensing/tracker code run before/without
%   eval/config/evalParams.m being final.
if ischar(names), names = {names}; end
v = default;
if nargin < 1 || isempty(params) || ~isstruct(params), return, end
for n = 1:numel(names)
    if isfield(params, names{n}) && ~isempty(params.(names{n}))
        v = params.(names{n}); return
    end
end
fn = fieldnames(params);
for f = 1:numel(fn)
    s = params.(fn{f});
    if isstruct(s) && isscalar(s)
        for n = 1:numel(names)
            if isfield(s, names{n}) && ~isempty(s.(names{n}))
                v = s.(names{n}); return
            end
        end
    end
end
end
