"""Summarize native controls without treating synthetic noise as real sensor noise."""
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parent
result=[]
for filename in ('probe-results.jsonl','mesh-probe-results.jsonl'):
    for line in (ROOT/filename).read_text().splitlines():
        record=json.loads(line);summary={k:record[k] for k in ('run','side')};summary['source']=filename
        for mode,values in record.items():
            if not isinstance(values,dict):continue
            summary[mode]={'trials':values['trials'],'rejected':values['rejected']}
            for key in ('depth_error_mm','position_error_mm','normal_error_deg'):
                a=np.array([v[key] for v in values['measurements']])
                summary[mode][key]=dict(mean=float(a.mean()),std=float(a.std()),max_abs=float(abs(a).max())) if len(a) else None
        result.append(summary)
        print(Path(record['run']).name,{mode:round(summary[mode]['depth_error_mm']['mean'],3) if summary[mode]['depth_error_mm'] else 'rejected' for mode in ('ideal','measured','visible_mesh') if mode in summary})
(ROOT/'probe-summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
