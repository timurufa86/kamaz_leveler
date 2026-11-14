import io,sys,re,hashlib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
P='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
t=open(P,encoding='utf-8').read()
pat=re.compile(r'\(fp_text user "[^"]*" \(at (-?[\d.]+) (-?[\d.]+) 90\) \(layer "F\.SilkS"\) '
               r'\(effects \(font \(size ([\d.]+) ([\d.]+)\) \(thickness ([\d.]+)\)\)\)\)')
def fix(m):
    x,y,s1,s2,th=m.group(1),float(m.group(2)),m.group(3),m.group(4),m.group(5)
    # angle 90: "left" extends toward -y, "right" toward +y
    just='right' if y>0 else 'left'   # row1 y=+1.02 -> extend +y ; row2 y=-23.88 -> extend -y
    old=m.group(0)
    new=old[:-3]+' (justify %s)))'%just
    return new
t2,n=pat.subn(fix,t)
print('labels patched:',n)
open(P,'w',encoding='utf-8',newline='').write(t2)
print('sha256', hashlib.sha256(open(P,'rb').read()).hexdigest())
print('parens', t2.count('(')-t2.count(')'), '| bytes', len(t), '->', len(t2))
