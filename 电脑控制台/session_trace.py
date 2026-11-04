"""Append-only navigation diagnostics. No hardware access."""
from pathlib import Path
from datetime import datetime
from dataclasses import asdict,is_dataclass
import json,time

class SessionTrace:
 def __init__(self,directory):
  root=Path(directory);root.mkdir(parents=True,exist_ok=True)
  self.path=root/('导航_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.jsonl')
  self.file=self.path.open('x',encoding='utf-8');self.write('session',dict(client='97F4/R6'))
 def write(self,kind,data):
  if self.file.closed:return
  def convert(v):return asdict(v) if is_dataclass(v) else repr(v)
  self.file.write(json.dumps(dict(time=time.monotonic(),event=kind,data=data),ensure_ascii=False,default=convert)+'\n');self.file.flush()
 def close(self):self.file.close()
