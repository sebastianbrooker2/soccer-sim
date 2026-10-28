function s = radiusTag(r)
%RADIUSTAG 0.25 -> '0p25', 1 -> '1p0' (column-name suffix for within-radius metrics).
s = sprintf('%g', r);
if ~contains(s, '.'), s = [s '.0']; end
s = strrep(s, '.', 'p');
end
