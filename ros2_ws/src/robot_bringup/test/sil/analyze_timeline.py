"""Summarize wall-clock SIL evidence without logging control tokens."""
import argparse
import json
from pathlib import Path


def percentile(values, fraction):
    values=sorted(values)
    return round(values[min(len(values)-1,int((len(values)-1)*fraction))],3) if values else None


def analyze(path):
    rows=[json.loads(line) for line in Path(path).read_text().splitlines()]
    result={}
    for name in ('native_status_tx','control_state_rx','/motor/host_command_array',
                 '/robot/managed/motor/command_array','wire_command_rx'):
        events=[r for r in rows if r['event']==name]
        gaps=[(b['mono_ns']-a['mono_ns'])/1e6 for a,b in zip(events,events[1:])]
        source_gaps=[(b['source_ns']-a['source_ns'])/1e6 for a,b in zip(events,events[1:])
            if a.get('source_ns') and b.get('sequence')==a.get('sequence',-1)+1]
        ages=[(r['wall_ns']-r['source_ns'])/1e6 for r in events if r.get('source_ns')]
        entry=dict(count=len(events),receipt_gap_max_ms=percentile(gaps,1),
            consecutive_source_gap_max_ms=percentile(source_gaps,1),
            source_age_p99_ms=percentile(ages,.99),source_age_max_ms=percentile(ages,1))
        if name=='wire_command_rx':
            entry['deadline_remaining_min_ms']=min((r['deadline']-r['tick'] for r in events),default=None)
            entry['rejected']=sum(not r['accepted'] for r in events)
        result[name]=entry
    gaps=[r['gap_ms'] for r in rows if r['event']=='native_service_gap']
    result['native_service_gap_ms']=dict(p99=percentile(gaps,.99),maximum=percentile(gaps,1))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('timeline',type=Path)
    args=parser.parse_args()
    print(json.dumps(analyze(args.timeline),ensure_ascii=False,indent=2))
