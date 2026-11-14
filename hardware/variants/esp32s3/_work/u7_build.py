import io,sys,re,hashlib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
BAK='_work/BACKUP_U7_socket.kicad_mod'
OUT='kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod'
src=open(BAK,encoding='utf-8').read()

MOVES=[('(fp_text user "ANTENNA END" (at 0.6 -11.43)','(fp_text user "ANTENNA END" (at -3.2 -9.8)'),
       ('(fp_text user "J1" (at 2.6 -2.2)','(fp_text user "J1" (at -1.9 -2.2)'),
       ('(fp_text user "J3" (at 2.6 -21.0)','(fp_text user "J3" (at -1.9 -20.7)')]
for old,new in MOVES:
    assert src.count(old)==1, old
    src=src.replace(old,new)

label=re.compile(r'^  \(fp_text user "[^"]*" \(at -?[\d.]+ (-?[\d.]+) 90\) \(layer "F\.SilkS"\)'
                 r' \(effects \(font \(size [\d.]+ [\d.]+\) \(thickness ([\d.]+)\)\)\)\)\s*$')
tail=re.compile(r'\(thickness ([\d.]+)\)\)\)\)\s*$')
outl=[]; n=0
for line in src.split('\n'):
    m=label.match(line)
    if m and 'justify' not in line:
        just='right' if float(m.group(1))>0 else 'left'
        newline=tail.sub(r'(thickness \1)) (justify %s)))'%just, line)
        assert newline!=line
        outl.append(newline); n+=1
    else:
        outl.append(line)
print('labels justified:',n); assert n==44
t2='\n'.join(outl)
open(OUT,'w',encoding='utf-8',newline='').write(t2)
print('parens:',t2.count('(')-t2.count(')'),'| quotes even:',t2.count('"')%2==0)
print('sha256',hashlib.sha256(open(OUT,'rb').read()).hexdigest())
pA=[l for l in open(BAK,encoding='utf-8') if l.startswith('  (pad ')]
pB=[l for l in open(OUT,encoding='utf-8') if l.startswith('  (pad ')]
print('pad lines identical:',pA==pB,len(pB))
