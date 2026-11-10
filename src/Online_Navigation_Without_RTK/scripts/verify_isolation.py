#!/usr/bin/env python3
from pathlib import Path
import os,json
root=Path(__file__).resolve().parents[1];old=Path('/home/potato/Slicer_Workspace')
before=json.loads((root/'migration/old_workspace_before.json').read_text());now={}
for d,dirs,files in os.walk(old,followlinks=False):
    for name in dirs+files:
        p=Path(d)/name;s=p.lstat();now[str(p.relative_to(old))]=[s.st_mode,s.st_size,s.st_mtime_ns,s.st_ctime_ns,os.readlink(p) if p.is_symlink() else None]
report=dict(old_root=str(old),baseline_entries=len(before),current_entries=len(now),changed=sorted(k for k in before.keys()&now.keys() if before[k]!=now[k]),added=sorted(now.keys()-before.keys()),removed=sorted(before.keys()-now.keys()))
report['unchanged']=not(report['changed'] or report['added'] or report['removed'])
(root/'migration/isolation_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));raise SystemExit(0 if report['unchanged'] else 1)
