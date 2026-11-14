import io,sys,hashlib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
P='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
src=open(P,encoding='utf-8').read()
EDITS=[
 ('(fp_text user "ANTENNA END" (at 0.6 -11.43)',
  '(fp_text user "ANTENNA END" (at -3.2 -9.8)'),
 ('(fp_text user "J1" (at 2.6 -2.2)',
  '(fp_text user "J1" (at -1.9 -2.2)'),
 ('(fp_text user "J3" (at 2.6 -21.0)',
  '(fp_text user "J3" (at -1.9 -20.7)'),
]
out=src
for old,new in EDITS:
    n=out.count(old)
    assert n==1, 'occurrences=%d for %r'%(n,old)
    out=out.replace(old,new)
    print('OK 1x:', old.strip())
open(P,'w',encoding='utf-8',newline='').write(out)
print('sha256 after ', hashlib.sha256(open(P,'rb').read()).hexdigest())
print('bytes before/after', len(src), len(out))
