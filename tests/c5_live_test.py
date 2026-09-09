"""Live C5 finite-burst test. TX is opt-in because it emits RF."""
import argparse
import json
import numpy as np
import SoapySDR as S

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',default='/dev/ttyACM0')
p.add_argument('--wire-bits',type=int,choices=[8,10],default=10)
p.add_argument('--tx',action='store_true')
a=p.parse_args()
S.loadModules()
d=S.Device('driver=espsdr,serial='+a.port)
assert d.getHardwareKey()=='ESP32-C5'
d.writeSetting('WIRE_BITS',str(a.wire_bits))
assert d.readSetting('WIRE_BITS')==str(a.wire_bits)
for value in ['0','9','16','8x']:
 try:d.writeSetting('WIRE_BITS',value)
 except RuntimeError:pass
 else:raise AssertionError('invalid WIRE_BITS accepted')
assert not d.getFullDuplex(S.SOAPY_SDR_RX,0)
report=[]
for fmt,dtype in ((S.SOAPY_SDR_CF32,np.complex64),(S.SOAPY_SDR_CS16,np.int16)):
 rx=d.setupStream(S.SOAPY_SDR_RX,fmt)
 try:
  assert d.activateStream(rx)==S.SOAPY_SDR_NOT_SUPPORTED
  for rate in d.listSampleRates(S.SOAPY_SDR_RX,0):
   d.setSampleRate(S.SOAPY_SDR_RX,0,rate)
   assert d.activateStream(rx,0,0,16380)==0
   count=0
   while count<16380:
    b=np.empty(1000 if dtype==np.complex64 else 2000,dtype)
    r=d.readStream(rx,[b],1000,timeoutUs=1000000)
    assert r.ret>0,r
    if a.wire_bits==8:
     if dtype==np.complex64:
      assert np.all(b[:r.ret].real*128==np.rint(b[:r.ret].real*128))
      assert np.all(b[:r.ret].imag*128==np.rint(b[:r.ret].imag*128))
     else:assert np.all((b[:r.ret*2]%256==0)|(b[:r.ret*2]==32767))
    count+=r.ret
    assert bool(r.flags&S.SOAPY_SDR_END_BURST)==(count==16380)
   assert d.readStream(rx,[b],1000,timeoutUs=1000).ret==S.SOAPY_SDR_TIMEOUT
   report.append(dict(direction='rx',format=fmt,rate=rate,samples=count))
 finally:d.closeStream(rx)
if a.tx:
 tx=d.setupStream(S.SOAPY_SDR_TX,S.SOAPY_SDR_CF32)
 rx=d.setupStream(S.SOAPY_SDR_RX,S.SOAPY_SDR_CF32)
 try:
  z=(.25*np.exp(2j*np.pi*np.arange(16380)/16)).astype(np.complex64)
  for rate in d.listSampleRates(S.SOAPY_SDR_TX,0):
   d.setSampleRate(S.SOAPY_SDR_TX,0,rate)
   assert d.activateStream(tx)==0
   assert d.activateStream(rx,0,0,256)==S.SOAPY_SDR_STREAM_ERROR
   assert d.writeStream(tx,[z],len(z),0,timeoutUs=1000000).ret==S.SOAPY_SDR_NOT_SUPPORTED
   r=d.writeStream(tx,[z],len(z),S.SOAPY_SDR_END_BURST,timeoutUs=1000000)
   assert r.ret==len(z),r
   report.append(dict(direction='tx',rate=rate,samples=r.ret))
   assert d.activateStream(rx,0,0,256)==0
   b=np.empty(256,np.complex64)
   r=d.readStream(rx,[b],256,timeoutUs=1000000)
   assert r.ret==256,r
 finally:d.closeStream(tx);d.closeStream(rx)
print(json.dumps(report,indent=2))

if a.tx:
 tx=d.setupStream(S.SOAPY_SDR_TX,S.SOAPY_SDR_CS16)
 try:
  d.setSampleRate(S.SOAPY_SDR_TX,0,1000000)
  z=.25*np.exp(2j*np.pi*np.arange(4096)/16)
  b=np.empty(8192,np.int16)
  b[0::2]=np.rint(z.real*32768).astype(np.int16)
  b[1::2]=np.rint(z.imag*32768).astype(np.int16)
  assert d.activateStream(tx)==0
  assert d.writeStream(tx,[b],4096,S.SOAPY_SDR_END_BURST,timeoutUs=1000000).ret==4096
  print('CS16 TX: 4096 samples passed')
 finally:d.closeStream(tx)

if a.tx:
 tx=d.setupStream(S.SOAPY_SDR_TX,S.SOAPY_SDR_CF32)
 try:
  for rate in (6000000,40000000,80000000):
   d.setSampleRate(S.SOAPY_SDR_TX,0,rate)
   d.writeSetting('TX_REPEATS','16')
   assert d.readSetting('TX_REPEATS')=='16'
   z=(.2*np.exp(2j*np.pi*np.arange(4096)/256)).astype(np.complex64)
   assert d.activateStream(tx)==0
   assert d.writeStream(tx,[z],4096,S.SOAPY_SDR_END_BURST,timeoutUs=1000000).ret==4096
  d.writeSetting('TX_REPEATS','1')
  print('Cyclic TX: 16 repeats at 6, 40 and 80 MS/s passed')
 finally:d.closeStream(tx)
