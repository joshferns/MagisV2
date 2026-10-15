#!/usr/bin/env python3
"""Generates the ubloxBusInit lists in src/main/io/gps.cpp ( gps-spi-i2c task 4 ).

python3 ubx_frames_gen.py           print the frames
python3 ubx_frames_gen.py out.txt   also write the C block that was pasted into gps.cpp

VALSET keys from the u-blox M9/M10 interface description ( cross-checked with the SparkFun
u-blox_config_keys.h table ): CFG-SPIOUTPROT-UBX 0x107A0001, -NMEA 0x107A0002, CFG-I2COUTPROT-UBX
0x10720001, -NMEA 0x10720002, CFG-MSGOUT-UBX_NAV_PVT_SPI 0x2091000A, _I2C 0x20910006,
CFG-RATE-MEAS 0x30210001, CFG-RATE-NAV 0x30210002, CFG-NAVSPG-DYNMODEL 0x20110021 ( 6 = airborne < 1 g ).
"""
import struct
def frame(cls,mid,payload):
    b=bytes([cls,mid])+struct.pack('<H',len(payload))+payload
    a=bb=0
    for x in b: a=(a+x)&255; bb=(bb+a)&255
    return bytes([0xB5,0x62])+b+bytes([a,bb])
def valset(pairs):
    p=bytes([0,1,0,0])  # version 0, layer RAM, reserved
    for k,v,n in pairs: p+=struct.pack('<I',k)+v.to_bytes(n,'little')
    return frame(0x06,0x8A,p)
def cfg_prt(port,mode):
    return frame(0x06,0x00,struct.pack('<BBHIIHHHH',port,0,0,mode,0,0x0007,0x0001,0,0))
def cfg_msg(c,i,r): return frame(0x06,0x01,bytes([c,i,r]))
def cfg_rate(ms): return frame(0x06,0x08,struct.pack('<HHH',ms,1,1))
def nav5_dyn(m): return frame(0x06,0x24,struct.pack('<HB',0x0001,m)+bytes(33))
def show(name,frames):
    print(f"// {name}")
    for c,f in frames:
        print("  "+", ".join(f"0x{x:02X}" for x in f)+",    // "+c)
spi=[("CFG-VALSET RAM: SPI out UBX on / NMEA off, NAV-PVT on SPI every fix, 100 ms, AIR1",
      valset([(0x107a0001,1,1),(0x107a0002,0,1),(0x2091000a,1,1),(0x30210001,100,2),(0x30210002,1,2),(0x20110021,6,1)])),
     ("CFG-PRT SPI ( port 4 ): mode 0, ffCnt 50, in UBX+NMEA+RTCM, out UBX", cfg_prt(4,0x00003200)),
     ("CFG-MSG NAV-PVT every fix", cfg_msg(1,7,1)),
     ("CFG-MSG NAV-POSLLH every fix", cfg_msg(1,2,1)),
     ("CFG-MSG NAV-STATUS every fix", cfg_msg(1,3,1)),
     ("CFG-MSG NAV-SOL every fix", cfg_msg(1,6,1)),
     ("CFG-MSG NAV-VELNED every fix", cfg_msg(1,0x12,1)),
     ("CFG-RATE 100 ms ( 10 Hz ), 1 cycle, GPS time", cfg_rate(100)),
     ("CFG-NAV5 dynamic model only: 6 airborne < 1 g", nav5_dyn(6))]
i2c=[("CFG-VALSET RAM: I2C out UBX on / NMEA off, NAV-PVT on I2C every fix, 200 ms, AIR1",
      valset([(0x10720001,1,1),(0x10720002,0,1),(0x20910006,1,1),(0x30210001,200,2),(0x30210002,1,2),(0x20110021,6,1)])),
     ("CFG-PRT DDC ( port 0 ): address 0x42, in UBX+NMEA+RTCM, out UBX", cfg_prt(0,0x42<<1)),
     ("CFG-MSG NAV-PVT every fix ( PVT only on I2C )", cfg_msg(1,7,1)),
     ("CFG-RATE 200 ms ( 5 Hz ), 1 cycle, GPS time", cfg_rate(200)),
     ("CFG-NAV5 dynamic model only: 6 airborne < 1 g", nav5_dyn(6))]
show("SPI",spi); print(sum(len(f) for _,f in spi)); show("I2C",i2c); print(sum(len(f) for _,f in i2c))

def carray(frames):
    out=[]
    for c,f in frames:
        out.append(f"    // {c}")
        for i in range(0,len(f),12):
            out.append("    "+", ".join(f"0x{x:02X}" for x in f[i:i+12])+",")
    return "\n".join(out)
import sys
if len(sys.argv)>1:
    block=f'''  #ifdef GPS_ON_BUS
// Configuration for a u-blox on SPI or I2C ( generated and checked by the topic's
// tests/ubx_frames_check.py ). Each list is sent once per ( re )configuration, one bus write at
// a time. CFG-VALSET ( RAM layer ) is for M9 / M10; the legacy CFG-PRT / MSG / RATE / NAV5
// frames are for M8. A module NAKs the set it does not support ( counted in GPS_ubxNakCount ).
// The CFG-PRT and VALSET frames switch the port's output to UBX only, so no NMEA disables are
// needed. Dynamic model: airborne < 1 g.
    #ifdef GPS_SPI
static const uint8_t ubloxBusInit [] = {{
{carray(spi)}
}};
    #else
// I2C carries NAV-PVT only, at 5 Hz: 500 B/s against the 800 B/s the 10 ms / 8 B poll can read.
static const uint8_t ubloxBusInit [] = {{
{carray(i2c)}
}};
    #endif
  #endif

'''
    open(sys.argv[1],'w').write(block)
