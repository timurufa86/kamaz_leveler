import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
NEW = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\kamaz_leveler_s3.kicad_pcb"
OLD = r"C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3\_work\BACKUP_session_start.kicad_pcb"
def per(path):
    b = pcbnew.LoadBoard(path)
    d = {}
    for t in b.GetTracks():
        nm = t.GetNetname()
        isv = isinstance(t, pcbnew.PCB_VIA)
        e = d.setdefault(nm, [0,0,0.0])
        if isv: e[1] += 1
        else:
            e[0] += 1
            s,en = t.GetStart(), t.GetEnd()
            e[2] += ((s.x-en.x)**2 + (s.y-en.y)**2)**0.5/1e6
    return d
a = per(OLD); c = per(NEW)
print('%-14s %18s %18s %12s' % ('net', 'OLD trk/via/len', 'NEW trk/via/len', 'dlen mm'))
for nm in sorted(set(a) | set(c)):
    oa = a.get(nm, [0,0,0.0]); ob = c.get(nm, [0,0,0.0])
    if oa[0]==ob[0] and oa[1]==ob[1] and abs(oa[2]-ob[2]) < 0.01: continue
    print('%-14s %6d/%3d/%8.2f %6d/%3d/%8.2f %+10.2f' % (nm, oa[0],oa[1],oa[2], ob[0],ob[1],ob[2], ob[2]-oa[2]))
print()
print('totals OLD seg=%d via=%d ; NEW seg=%d via=%d' % (sum(v[0] for v in a.values()), sum(v[1] for v in a.values()), sum(v[0] for v in c.values()), sum(v[1] for v in c.values())))
