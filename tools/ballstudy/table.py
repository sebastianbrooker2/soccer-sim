#!/usr/bin/env python3
"""Print markdown table from study_*.csv files (b_=ball, o_=opponent). usage: table.py file.csv ..."""
import csv, sys
cols = ['b_avail','b_in95','b_in95cond','b_rmse','b_med','b_excess','b_multi','b_area','b_reacq','b_reacqFail','o_in95','o_rmse','o_false','o_reacq']
print('| variant | ' + ' | '.join(cols) + ' |'); print('|' + '---|'*(len(cols)+1))
seen = set()
for f in sys.argv[1:]:
    for r in csv.DictReader(open(f)):
        if r['label'] in seen: continue
        seen.add(r['label'])
        print('| ' + r['label'] + ' | ' + ' | '.join('%.2f' % float(r[c]) for c in cols) + ' |')
