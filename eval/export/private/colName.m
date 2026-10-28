function s = colName(tracker, cls)
%COLNAME CSV/pgfplots-safe column name for a tracker/class series.
s = char(matlab.lang.makeValidName(sprintf('%s_%s', char(tracker), char(cls))));
s = strrep(s, '__', '_');
end
