#!/usr/bin/env python3
# Mecanum-Mixer via I2C (pulses -127..+127) + Ramp
import smbus2

I2C_PORT=1
ADDR=0x34
REGS=[51,52,53,54]  # M1..M4

FWD   = [-1, +1, +1, -1]
LEFT  = [+1, +1, +1, +1]
RIGHT = [-1, -1, -1, -1]
CW    = [-1, +1, -1, +1]
CCW   = [+1, -1, +1, -1]

def _clip(x:int)->int:
    return 127 if x>127 else (-127 if x<-127 else x)

def _mix3(p1,p2,p3):
    return [_clip(p1[i]+p2[i]+p3[i]) for i in range(4)]

class MotorController:
    def __init__(self, ramp_step=3, g_vy=0.25):
        self._prev=[0,0,0,0]
        self._ramp=ramp_step
        self.G_VY=g_vy

    def _write(self, idx:int, val:int):
        with smbus2.SMBus(I2C_PORT) as bus:
            bus.write_i2c_block_data(ADDR, REGS[idx], [val & 0xFF])

    def stop_all(self):
        for i in range(4): self._write(i, 0)
        self._prev=[0,0,0,0]

    def base_mag_from_speed(self, forward_mm_s: float):
        return int(self.G_VY * forward_mm_s)

    def command(self, base_fwd=0, strafe=0, yaw=0):
        p_base  = [v*base_fwd for v in FWD] if base_fwd else [0,0,0,0]
        p_stra  = (LEFT if strafe>0 else RIGHT) if strafe!=0 else [0,0,0,0]
        p_stra  = [v*abs(strafe) for v in p_stra]
        p_yaw   = (CCW if yaw>0 else CW) if yaw!=0 else [0,0,0,0]
        p_yaw   = [v*abs(yaw) for v in p_yaw]
        out = _mix3(p_base, p_stra, p_yaw)

        sm=[]
        for i in range(4):
            cur=self._prev[i]; tgt=out[i]
            if tgt>cur+self._ramp: cur+=self._ramp
            elif tgt<cur-self._ramp: cur-=self._ramp
            else: cur=tgt
            sm.append(cur)

        for i,val in enumerate(sm):
            self._write(i,val)
        self._prev=sm