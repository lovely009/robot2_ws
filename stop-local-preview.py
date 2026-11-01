"""Stop only the recorded preview parent after checking its current identity."""
import json
import os
from pathlib import Path
import signal
import time

root=Path(__file__).resolve().parent
pid=json.loads((root/'runtime/services.json').read_text())['parent']
proc=Path('/proc')/str(pid)
if not proc.exists():
    print('Preview already stopped')
else:
    command=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode()
    if 'local_sil_preview.py' not in command or (proc/'cwd').resolve()!=root:
        raise SystemExit('Recorded PID belongs to another process; left untouched')
    os.kill(pid,signal.SIGTERM)
    for _ in range(100):
        if not proc.exists():break
        time.sleep(.1)
    print('Preview stop requested; all owned children are cleaned by its parent')
