"""Execute assembled Tramp TX instructions and verify 8N1 and wire release."""
import re
from pathlib import Path
root=Path(__file__).resolve().parents[1]
header=(root/'build-release/tramp.pio.h').read_text()
words=header.split('tramp_program_instructions[] = {')[1].split('};')[0]
code=[int(v,16) for v in re.findall(r'0x([0-9a-fA-F]{4})',words)]
for value in range(256):
    data=iter([value]*16)
    pc=x=y=osr=0
    line=1
    trace=[]
    starts=[]
    for step in range(1000):
        w=code[pc]; op=w>>13; dest=(w>>5)&7; arg=w&31
        cycles=1+((w>>8)&31); nxt=pc+1
        if op==7:
            if dest==4:
                if not arg: break
            elif dest==1: x=arg
            elif dest==2: y=arg
            elif dest==0:
                line=arg
                if not arg: starts.append(len(trace))
            else: raise AssertionError(hex(w))
        elif op==4: osr=next(data)
        elif op==3:
            assert dest==0 and arg==1
            line=osr&1;osr>>=1
        elif op==0:
            if dest==2:
                if x: nxt=arg
                x=(x-1)&0xffffffff
            elif dest==4:
                if y: nxt=arg
                y=(y-1)&0xffffffff
            else: raise AssertionError(hex(w))
        else: raise AssertionError(hex(w))
        trace.extend([line]*cycles);pc=nxt
    else: raise AssertionError('No wire release')
    assert len(starts)==16
    for start in starts:
        expected=[0]+[(value>>b)&1 for b in range(8)]+[1]
        assert [trace[start+4+b*8] for b in range(10)]==expected
    assert len(trace)>=starts[-1]+80, 'Wire released before stop bit completed'
print('Tramp assembled TX: all 256 bytes, 16-byte frames, 9600 8N1, stop and wire release: PASS')
