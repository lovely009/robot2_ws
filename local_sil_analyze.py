"""Report source clocks, DDS receipts, and native output pump clocks separately."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent

def read(path):return [json.loads(line) for line in path.read_text().splitlines()]

def stats(values):
    values=sorted(values)
    def q(f):return round(values[int((len(values)-1)*f)],6) if values else None
    return dict(count=len(values),minimum=q(0),p50=q(.5),p95=q(.95),p99=q(.99),maximum=q(1))

def intervals(rows,field='mono_ns'):
    return [(b[field]-a[field])/1e6 for a,b in zip(rows,rows[1:])]

def stream(rows):
    result=dict(count=len(rows),receipt_gap_ms=stats(intervals(rows)))
    if len(rows)>1:
        result['receipt_hz']=round((len(rows)-1)*1e9/(rows[-1]['mono_ns']-rows[0]['mono_ns']),4)
    stamped=[r for r in rows if r.get('source_ns')]
    if len(stamped)>1:
        result['source_gap_ms']=stats(intervals(stamped,'source_ns'))
        result['source_hz']=round((len(stamped)-1)*1e9/(stamped[-1]['source_ns']-stamped[0]['source_ns']),4)
        result['source_age_ms']=stats([(r['wall_ns']-r['source_ns'])/1e6 for r in stamped])
    return result

def analyze(folder):
    outcome=json.loads((folder/'outcome.json').read_text())
    trace=read(folder/'timeline.jsonl');outputs=read(folder/'outputs.jsonl')
    start=outcome['start_ns'];goal_end=outcome['goal_end_ns']
    soft=next((r for r in trace if r['event']=='native_status_tx' and
        start<r['mono_ns']<goal_end and r['execution']==5),None)
    # Warm-up is explicit, then retain everything up to the first reported
    # soft-stop; fault propagation and teardown cannot inflate these maxima.
    first_output=next(r['mono_ns'] for r in outputs if r['mono_ns']>=start)
    begin=first_output+100_000_000;end=soft['mono_ns'] if soft else goal_end
    active=[r for r in trace if begin<=r['mono_ns']<=end]
    result=dict(outcome=outcome,window=dict(warmup_ms=100,
        duration_seconds=round((end-begin)/1e9,6),end='first reported soft-stop' if soft else 'goal terminal'))
    names=('native_status_tx','control_state_rx','/motor/host_command_array',
        '/robot/managed/motor/command_array','wire_command_rx')
    result['streams']={name:stream([r for r in active if r['event']==name]) for name in names}
    wire=[r for r in active if r['event']=='wire_command_rx']
    result['wire']=dict(rejected=sum(not r['accepted'] for r in wire),
        deadline_remaining_ms=stats([r['deadline']-r['tick'] for r in wire]),
        target_counts=sorted(set(r['count'] for r in wire)))
    result['native_service_gaps_over_3ms']=stats([r['gap_ms'] for r in active if r['event']=='native_service_gap'])
    per_axis={i:[r for r in outputs if r['motor_id']==i and begin<=r['mono_ns']<=end] for i in range(1,38)}
    wall=[];virtual=[];rates=[];counts=[]
    for rows in per_axis.values():
        counts.append(len(rows));wall.extend(intervals(rows))
        virtual.extend(b['tick']-a['tick'] for a,b in zip(rows,rows[1:]))
        if len(rows)>1:rates.append((len(rows)-1)*1e9/(rows[-1]['mono_ns']-rows[0]['mono_ns']))
    result['outputs']=dict(covered_axes=sum(bool(v) for v in per_axis.values()),
        per_axis_count_min=min(counts),per_axis_count_max=max(counts),
        wall_hz_min=round(min(rates),4),wall_hz_max=round(max(rates),4),
        wall_gap_ms=stats(wall),virtual_gap_ms=stats(virtual),
        catchup_intervals_under_1ms=sum(v<1 for v in wall),overflow=outcome['output_overflow'])
    if soft:
        previous=[r for r in trace if r['event']=='wire_command_rx' and r['mono_ns']<soft['mono_ns']]
        following=[r for r in trace if r['event']=='wire_command_rx' and r['mono_ns']>soft['mono_ns']]
        gc=[r for r in trace if r['event']=='python_gc_generation2' and
            soft['mono_ns']-100_000_000<r['mono_ns']<=soft['mono_ns']]
        result['first_softstop']=dict(event=soft,previous_wire=previous[-1] if previous else None,
            next_wire=following[0] if following else None,gc_events=gc,
            gc_duration_ms=round((gc[-1]['mono_ns']-gc[0]['mono_ns'])/1e6,6) if len(gc)==2 else None)
    (folder/'summary.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    results={p.name:analyze(p) for p in sorted((ROOT/'evidence').iterdir()) if
        p.is_dir() and (p/'outputs.jsonl').exists()}
    (ROOT/'evidence/measurement-summary.json').write_text(json.dumps(results,indent=2))
    for name,r in results.items():
        print(name,'window',r['window'],'goal_passed',r['outcome'].get('goal_passed'),
            'gc_ms',r.get('first_softstop',{}).get('gc_duration_ms'))
        for stream_name,m in r['streams'].items():print(stream_name,m)
        print('outputs',r['outputs'])
