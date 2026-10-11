"""Non-blocking MOSS event sounds. Python 3.9+; standard library only.

emit() queues a cue, never drives the robot, and never waits for playback.
Backend completion means the playback process exited, not proof of audibility.
"""
import argparse
from collections import OrderedDict
import json
import math
from pathlib import Path
import queue
import random
import shutil
import subprocess
import tempfile
import threading
import time

ROOT=Path(__file__).resolve().parent

def playback_command(path, backend='auto', target=None, volume=.65):
    if not math.isfinite(volume) or not 0 <= volume <= 1:
        raise ValueError('Volume must be between 0 and 1')
    if backend=='auto':
        backend=next((x for x in ('pw-play','paplay','afplay') if shutil.which(x)),None)
    if backend not in ('pw-play','paplay','afplay') or not shutil.which(backend):
        raise RuntimeError('No supported playback backend; select pw-play, paplay, or afplay')
    if backend=='pw-play':
        cmd=[backend,f'--volume={volume:.3f}']
        if target:cmd.append(f'--target={target}')
    elif backend=='paplay':
        cmd=[backend,f'--volume={round(volume*65536)}','--client-name=MOSS','--stream-name=MOSS reaction']
        if target:cmd.append(f'--device={target}')
    else:
        if target:raise ValueError('afplay uses the macOS selected output; omit target')
        cmd=[backend,'-v',str(volume)]
    return cmd+[str(path)]

class MossSoundPlayer:
    def __init__(self, root=ROOT, config=None, dry_run=False):
        self.root=Path(root).resolve()
        self.manifest=json.loads((self.root/'manifest.json').read_text())
        self.config=json.loads((self.root/'config.json').read_text())
        if config:self.config.update(config)
        self.volume=float(self.config['volume'])
        if not math.isfinite(self.volume) or not 0<=self.volume<=1:raise ValueError('Invalid volume')
        self.rules={}
        for name,rule in self.manifest['events'].items():
            rule=dict(rule);rule.update(self.config.get('events',{}).get(name,{}))
            self.rules[name]=rule
        self.dry_run=dry_run;self.muted=bool(self.config['mute']);self.closed=False
        self.lock=threading.Condition();self.pending=[];self.current=None
        self.last_started={};self.last_variant={};self.last_end=-float('inf')
        self.ids=OrderedDict();self.status=queue.Queue(maxsize=128)
        self.rng=random.Random();self.worker=threading.Thread(target=self._run,name='moss-audio',daemon=True)
        self.worker.start()

    def _status(self, state, **details):
        row={'state':state,**details}
        try:self.status.put_nowait(row)
        except queue.Full:
            try:self.status.get_nowait()
            except queue.Empty:pass
            self.status.put_nowait(row)

    def drain_status(self):
        result=[]
        while True:
            try:result.append(self.status.get_nowait())
            except queue.Empty:return result

    def emit(self, event, event_id=None):
        """Return True if queued. False means suppressed; no playback promise.

        A busy audio worker causes this call to drop the cue rather than block.
        Call on confirmed state transitions, never on every telemetry frame.
        """
        if event not in self.rules:return False
        if event_id is not None and (not isinstance(event_id,str) or len(event_id)>200):return False
        if not self.lock.acquire(blocking=False):return False
        try:
            now=time.monotonic();rule=self.rules[event]
            if self.closed or self.muted or self.volume==0:return False
            if not rule['enabled'] and not self.config['personality']:return False
            if now-self.last_started.get(event,-float('inf'))<rule['cooldown_s']:return False
            self.pending=[p for p in self.pending if p['expires']>now]
            if any(p['event']==event for p in self.pending):return False
            while self.ids and next(iter(self.ids.values()))<=now:self.ids.popitem(last=False)
            key=(event,event_id) if event_id is not None else None
            if key is not None and key in self.ids:return False
            if len(self.pending)>=int(self.config['queue_limit']):
                low=min(self.pending,key=lambda p:p['priority'])
                if rule['priority']<=low['priority']:return False
                self.pending.remove(low)
            self.pending.append({'event':event,'priority':rule['priority'],'created':now,'expires':now+rule['ttl_s']})
            if key is not None:
                self.ids[key]=now+self.config['event_id_window_s']
                while len(self.ids)>int(self.config['dedup_limit']):self.ids.popitem(last=False)
            self.lock.notify();return True
        finally:self.lock.release()

    def set_mute(self, muted=True):
        with self.lock:
            self.muted=bool(muted)
            if self.muted:self.pending.clear()
            self.lock.notify()

    def set_volume(self, volume):
        if not math.isfinite(volume) or not 0<=volume<=1:raise ValueError('Invalid volume')
        with self.lock:self.volume=volume;self.lock.notify()

    def _finish(self, state, terminate=False):
        current=self.current
        if current is None:return
        p=current['process']
        if terminate and p.poll() is None:
            p.terminate()
            try:p.wait(timeout=.25)
            except subprocess.TimeoutExpired:p.kill();p.wait(timeout=.5)
        error=''
        if state in ('failed','timeout'):
            current['stderr'].seek(0);error=current['stderr'].read(2048).decode('utf-8','replace')
        current['stderr'].close()
        self._status(state,event=current['event'],sound=current['sound'],returncode=p.poll(),error=error)
        self.current=None;self.last_end=time.monotonic()

    def _run(self):
        try:
            while True:
                with self.lock:
                    now=time.monotonic()
                    if self.closed:
                        self.pending.clear();self._finish('stopped',True);return
                    if self.muted or self.volume==0:
                        self.pending.clear();self._finish('muted',True);self.lock.wait(.025);continue
                    if self.current:
                        code=self.current['process'].poll()
                        if code is not None:self._finish('finished' if code==0 else 'failed')
                        elif now>=self.current['deadline']:self._finish('timeout',True)
                    fresh=[]
                    for item in self.pending:
                        if item['expires']>now:fresh.append(item)
                        else:self._status('expired',event=item['event'])
                    self.pending=fresh
                    item=min(self.pending,key=lambda p:(-p['priority'],p['created'])) if self.pending else None
                    # Critical battery feedback can interrupt a lower-priority cue.
                    if self.current and item and item['priority']>=90 and item['priority']>self.current['priority']:
                        self._finish('preempted',True)
                    if self.current or item is None:
                        self.lock.wait(.025);continue
                    if item['priority']<75 and now-self.last_end<self.config['global_gap_s']:
                        self.lock.wait(.025);continue
                    self.pending.remove(item);name=item['event'];rule=self.rules[name]
                    if now-self.last_started.get(name,-float('inf'))<rule['cooldown_s']:continue
                    options=[v for v in rule['variants'] if v!=self.last_variant.get(name)] or rule['variants']
                    sound=self.rng.choice(options);asset=self.manifest['assets'][sound]
                    path=(self.root/asset['path']).resolve()
                    if self.root not in path.parents or not path.is_file():
                        self._status('failed',event=name,error='Invalid or missing asset');continue
                    if self.dry_run:
                        self._status('simulated',event=name,sound=sound)
                        self.last_variant[name]=sound;self.last_started[name]=now;self.last_end=now;continue
                    err=None
                    try:
                        cmd=playback_command(path,self.config['backend'],self.config.get('target'),self.volume)
                        err=tempfile.TemporaryFile()
                        proc=subprocess.Popen(cmd,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=err)
                    except (OSError,ValueError,RuntimeError) as exc:
                        if err:err.close()
                        self._status('failed',event=name,error=str(exc));continue
                    self.current={**item,'sound':sound,'process':proc,'stderr':err,'deadline':time.monotonic()+asset['duration_s']+4}
                    self.last_variant[name]=sound;self.last_started[name]=now
                    self._status('started',event=name,sound=sound)
        except Exception as exc:
            # Audio failures stay within this optional subsystem.
            with self.lock:
                self.closed=True;self.pending.clear()
                try:self._finish('failed',True)
                finally:self._status('worker_failed',error=str(exc))

    def wait_idle(self, timeout=8):
        """Convenience for CLI/tests only; do not call from motion control."""
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            with self.lock:
                if not self.pending and self.current is None:return True
            time.sleep(.02)
        return False

    def close(self):
        with self.lock:self.closed=True;self.lock.notify()
        self.worker.join(timeout=2)

    def __enter__(self):return self
    def __exit__(self,*args):self.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['auto','pw-play','paplay','afplay'])
    parser.add_argument('--target',help='Exact PipeWire node name or PulseAudio sink name')
    parser.add_argument('--volume',type=float)
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--personality',action='store_true',help='Enable optional decorative events')
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('list');p=sub.add_parser('play');p.add_argument('event')
    p=sub.add_parser('audition');p.add_argument('sound')
    sub.add_parser('demo');sub.add_parser('listen')
    args=parser.parse_args();manifest=json.loads((ROOT/'manifest.json').read_text())
    if args.command=='list':
        for name,rule in manifest['events'].items():print(f"{name:24} {'on' if rule['enabled'] else 'optional':8} {rule['description']}")
        return 0
    config=json.loads((ROOT/'config.json').read_text())
    for key in ('backend','target','volume'):
        if getattr(args,key) is not None:config[key]=getattr(args,key)
    if args.personality:config['personality']=True
    if args.command in ('demo','audition'):
        if args.command=='demo':path=ROOT/'preview/MOSS_Character_Demo.wav';timeout=40
        else:
            if args.sound not in manifest['assets']:parser.error('Unknown sound ID')
            asset=manifest['assets'][args.sound];path=ROOT/asset['path'];timeout=asset['duration_s']+4
        cmd=playback_command(path,config['backend'],config.get('target'),config['volume'])
        if args.dry_run:print(json.dumps({'state':'dry_run_command','command':cmd}));return 0
        try:return subprocess.run(cmd,timeout=timeout,check=False).returncode
        except subprocess.TimeoutExpired:print('Audio backend timed out');return 1
    if args.command=='play':
        if args.event not in manifest['events']:parser.error('Unknown event')
        config['events']={**config.get('events',{}),args.event:{'enabled':True}}
    failed=False
    with MossSoundPlayer(ROOT,config,args.dry_run) as player:
        if args.command=='play':
            accepted=player.emit(args.event);print(json.dumps({'accepted':accepted,'event':args.event}),flush=True)
            failed=not accepted
        else:
            import sys
            for line in sys.stdin:
                try:
                    obj=json.loads(line)
                    if not isinstance(obj,dict):raise ValueError('Expected a JSON object')
                    if obj.get('command')=='mute':player.set_mute(bool(obj.get('value',True)));continue
                    if obj.get('command')=='volume':player.set_volume(float(obj['value']));continue
                    event=obj['event'];accepted=player.emit(event,obj.get('id'))
                    print(json.dumps({'accepted':accepted,'event':event}),flush=True)
                except (ValueError,KeyError,TypeError) as exc:print(json.dumps({'input_error':str(exc)}),flush=True)
                for row in player.drain_status():print(json.dumps(row),flush=True)
        if not player.wait_idle(8):failed=True
        for row in player.drain_status():
            print(json.dumps(row),flush=True)
            if row['state'] in ('failed','worker_failed','timeout'):failed=True
    return 1 if failed else 0

if __name__=='__main__':raise SystemExit(main())
