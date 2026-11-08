"""Native frame recording and onboard experiment command. Uses UDP 5005/5006/5007."""
from __future__ import annotations
from dataclasses import dataclass
import math
import struct
HEADER = struct.Struct('<8B3I6Qq2IH2B')
COMMAND = struct.Struct('<BBBI4f')
PHASES = ('IDLE','PRESTREAM','OFFBOARD','ARM','TAKEOFF','DWELL','LEG','LAND',
          'DONE','ABORT','REJECTED','PILOT_TAKEOVER')
@dataclass(frozen=True)
class NativeFrame:
    drone_id: int
    sensor: int
    side: int
    hz: int
    flags: int
    boot_id: int
    sequence: int
    queue_drops: int
    ready_before_us: int
    ready_after_us: int
    read_start_us: int
    read_end_us: int
    previous_not_ready_us: int
    send_us: int
    clock_offset_us: int
    sync_rtt_us: int
    sync_age_us: int
    transfer_bytes: int
    streamcount: int
    sync_samples: int
    distances: tuple[int, ...]
    statuses: tuple[int, ...]
    @property
    def sync_locked(self):
        return bool(self.flags & 1) and self.sync_age_us <= 2_000_000 and self.sync_rtt_us <= 5000
    @property
    def frame_reference_us(self):
        if self.flags & 2 and not self.flags & 4:
            return (self.previous_not_ready_us+self.ready_after_us)//2
        return self.read_end_us
    @property
    def reference_method(self):
        return 'ready_bracket_midpoint' if self.flags & 2 and not self.flags & 4 else 'read_end_unbounded'
    @property
    def readiness_halfwidth_us(self):
        if self.reference_method == 'ready_bracket_midpoint':
            return (self.ready_after_us-self.previous_not_ready_us+1)//2
        return None

def decode_native(data: bytes) -> NativeFrame | None:
    if len(data)<HEADER.size: return None
    f=HEADER.unpack_from(data)
    kind,version,drone,sensor,rows,cols,hz,flags=f[:8]
    n=rows*cols
    if kind!=5 or version!=1 or sensor>=8 or rows not in (4,8) or cols!=rows: return None
    if not 1<=hz<=(15 if rows==8 else 60) or len(data)!=HEADER.size+n*3: return None
    if flags & ~7: return None
    boot,seq,drops,rb,ra,rs,re,previous,sent,offset,rtt,age,size,stream,samples=f[8:]
    if not 0<rb<=ra<=rs<=re<=sent: return None
    if flags&2 and not 0<previous<=rb: return None
    distances=struct.unpack_from(f'<{n}h',data,HEADER.size)
    statuses=struct.unpack_from(f'<{n}B',data,HEADER.size+n*2)
    return NativeFrame(drone,sensor,rows,hz,flags,boot,seq,drops,rb,ra,rs,re,previous,
                       sent,offset,rtt,age,size,stream,samples,distances,statuses)

# Firmware mission codes (main/experiment_task.h). 'rectangle-centred' takes
# off at the middle of the bottom (south) edge and needs firmware with mode 3.
MODE_CODES={'takeoff-land':1,'rectangle':2,'rectangle-centred':3}

def mission_waypoints(mode: str, width=1., length=1.):
    """(north, east) waypoints relative to takeoff, as experiment_waypoint()
    in main/experiment_plan.c flies them. First = takeoff, last = landing."""
    if mode=='rectangle':
        return [(0,0),(length,0),(length,width),(0,width),(0,0)]
    if mode=='rectangle-centred':
        h=width/2
        return [(0,0),(0,-h),(length,-h),(length,h),(0,h),(0,0)]
    return [(0,0)]

def build_experiment(mission_id: int, mode: str, width=1., length=1., altitude=.5, speed=.3):
    if not 0<mission_id<2**32 or mode not in MODE_CODES:
        raise ValueError('Invalid mission ID or mode')
    for value,low,high in ((width,.2,10),(length,.2,10),(altitude,.3,2),(speed,.05,.5)):
        if not math.isfinite(value) or not low<=value<=high: raise ValueError('Mission parameters outside firmware limits')
    return COMMAND.pack(2,7,MODE_CODES[mode],mission_id,width,length,altitude,speed)
