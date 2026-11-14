import io,sys,hashlib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
P='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
EDITS=[
 ('  (fp_text user "11" (at -40.64 1.02 90) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.12)) (justify right)))',
  '  (fp_text user "11" (at -40.64 1.02 90) (layer "F.SilkS") (effects (font (size 0.58 0.58) (thickness 0.12)) (justify right)))'),
 ('  (fp_text user "G" (at -53.34 1.02 90) (layer "F.SilkS") (effects (font (size 0.9 0.9) (thickness 0.12)) (justify right)))',
  '  (fp_text user "G" (at -53.34 1.02 90) (layer "F.SilkS") (effects (font (size 0.58 0.58) (thickness 0.12)) (justify right)))'),
]
t=open(P,encoding='utf-8').read()
for old,new in EDITS:
    assert t.count(old)==1, (t.count(old), old)
    t=t.replace(old,new)
    print('1x ->', new.strip()[:96])
open(P,'w',encoding='utf-8',newline='').write(t)
print('parens',t.count('(')-t.count(')'))
print('sha256',hashlib.sha256(open(P,'rb').read()).hexdigest())
