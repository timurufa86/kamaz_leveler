"""Run 3: surgical re-routes of the defective nets only (no global re-route)."""
import io
import sys
import math
import os
import time
sys.path.insert(0, r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work')
import pcbnew
import rr
import go
sys.stdout = go.sys.stdout
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass
from rr import Model, NX, NY, ix, iy, cx, cy
from go import route_net, width_of

ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = os.path.join(ROOT, 'kamaz_leveler_ruichi158.kicad_pcb')
DST = os.path.join(ROOT, '_work', 'fixed3a.kicad_pcb')
TARGETS = sys.argv[1].split(',') if len(sys.argv) > 1 else [
    'INT_MCU', 'VLV_IN3', 'VLV_IN4', 'VLV_IN5', 'VLV_IN6', 'BUT1',
    'ST_U3_34', 'ST_U4_34', '5V', '12V']

t0 = time.time()
m = Model(SRC)
m.build()
b = m.b
LAY = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 1}
LWR = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
segs_of, vias_of = {}, {}
for t in b.GetTracks():
    nm = t.GetNetname()
    segs_of.setdefault(nm, [])
    vias_of.setdefault(nm, [])
    if t.Type() == pcbnew.PCB_VIA_T:
        q = t.GetPosition()
        vias_of[nm].append((q.x / 1e6, q.y / 1e6))
    else:
        s0, e0 = t.GetStart(), t.GetEnd()
        segs_of[nm].append((s0.x / 1e6, s0.y / 1e6, e0.x / 1e6, e0.y / 1e6,
                            t.GetWidth() / 2e6, LAY[t.GetLayer()]))
rorder = sorted(segs_of.keys(), key=lambda n: -len(segs_of[n]))
wused = {n: round(2 * segs_of[n][0][4], 3) for n in rorder if segs_of[n]}
netnames = set(p['net'] for p in m.pads)


def allsegs(exclude):
    out = []
    for n in rorder:
        if n == exclude:
            continue
        for s in segs_of[n]:
            out.append((n, s[0], s[1], s[2], s[3], s[4], int(s[5])))
    return out


def _dseg(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
    return math.hypot(px - (x1 + t * dx), py - (y1 + t * dy))


def conflicts(segs, net):
    out = set()
    lst = allsegs(net)
    for (x1, y1, x2, y2, h, L) in segs:
        L = int(L)
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1)
        P = [(x1 + (x2 - x1) * t / n, y1 + (y2 - y1) * t / n) for t in range(n + 1)]
        mx0, mx1 = min(x1, x2) - h - 1.0, max(x1, x2) + h + 1.0
        my0, my1 = min(y1, y2) - h - 1.0, max(y1, y2) + h + 1.0
        for (sn, a, bb, c, e, hh, LL) in lst:
            if LL != L:
                continue
            if max(a, c) + hh < mx0 or min(a, c) - hh > mx1 or max(bb, e) + hh < my0 or min(bb, e) - hh > my1:
                continue
            lim = hh + h + rr.CLR - 0.01
            if min(_dseg(px, py, a, bb, c, e) for (px, py) in P) < lim:
                out.add(sn)
        for q in m.pads:
            if q['net'] == net or not (q['onF'] if L == 0 else q['onB']):
                continue
            if q['x'] + q['hx'] + h + rr.CLR < mx0 or q['x'] - q['hx'] - h - rr.CLR > mx1:
                continue
            if q['y'] + q['hy'] + h + rr.CLR < my0 or q['y'] - q['hy'] - h - rr.CLR > my1:
                continue
            for (px, py) in P:
                u = q['c'] * (px - q['x']) + q['s'] * (py - q['y'])
                v = -q['s'] * (px - q['x']) + q['c'] * (py - q['y'])
                qx = abs(u) - (q['hx'] - q['r'])
                qy = abs(v) - (q['hy'] - q['r'])
                sd = math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - q['r']
                if sd < rr.CLR + h - 0.01:
                    if q['net'] != '__none__':
                        out.add(q['net'])
                    break
    return out


log = []
done_nets = 0
for tgt in TARGETS:
    net = tgt if tgt in netnames else '/' + tgt
    if net not in netnames:
        print('unknown net %s' % tgt)
        continue
    fp = m.netpads(net)
    if len(fp) < 2:
        continue
    w0 = width_of(net)
    ws = [w0] if w0 <= 0.3 else [w0, 0.6, 0.5]
    solved = False
    for wtry in ws:
        C = set()

        def prox(n):
            best = 1e6
            for s in segs_of[n]:
                mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
                for p in fp:
                    d = math.hypot(mx - p['x'], my - p['y'])
                    if d < best:
                        best = d
            return best
        cand = [n for n in rorder if n != net and segs_of[n]]
        cand.sort(key=prox)
        k_idx = 0
        chunks = [0, 1, 2, 3, 5, 8, 13, 21]
        segs = None
        for it in range(len(chunks) + 3):
            m.reset_pen()
            for n in rorder:
                if n in C or n == net:
                    continue
                m.add_copper(segs_of[n], vias_of[n])
            segs, vias = route_net(m, net, wtry / 2.0, log, margin=12.0, via_cost=1.0, use_hub=False)
            if not segs:
                if k_idx >= len(chunks):
                    break
                add = chunks[k_idx]
                k_idx += 1
                got = [n for n in cand if n not in C][:max(1, add)]
                if not got:
                    break
                C |= set(got)
                continue
            conf = conflicts(segs, net) - C
            if not conf:
                break
            C |= conf
        if not segs:
            continue
        m.add_copper(segs, vias)
        ok2 = True
        newgeo = {}
        for n in [x for x in rorder if x in C]:
            s2, v2 = route_net(m, n, wused.get(n, 0.15) / 2.0, log, margin=12.0, via_cost=2.0)
            if not s2:
                ok2 = False
                break
            m.add_copper(s2, v2)
            newgeo[n] = (s2, v2)
        if not ok2:
            continue
        for n, gp in newgeo.items():
            segs_of[n] = gp[0]
            vias_of[n] = gp[1]
        segs_of[net] = segs
        vias_of[net] = vias
        if net not in rorder:
            rorder.append(net)
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        print('%-12s re-routed w=%.2f segs %2d vias %2d len %6.2f rip %d (%.0fs)' %
              (net, wtry, len(segs), len(vias), L, len(C), time.time() - t0), flush=True)
        solved = True
        done_nets += 1
        break
    if not solved:
        print('%-12s UNRESOLVED' % net, flush=True)

# --- antenna: drop non-U7-pad copper inside the strip -------------------------
ANT = (-13.0, 34.0, 13.0, 42.75)


def in_strip(t):
    x1, y1 = t.GetStart().x / 1e6, t.GetStart().y / 1e6
    x2, y2 = t.GetEnd().x / 1e6, t.GetEnd().y / 1e6
    h = t.GetWidth() / 2e6
    return (max(x1, x2) + h > ANT[0] and min(x1, x2) - h < ANT[2] and
            max(y1, y2) + h > ANT[1] and min(y1, y2) - h < ANT[3])


u7nets = set(p['net'] for p in m.pads if p['ref'] == 'U7' and p['net'] != '__none__')
removed = []
for t in list(b.GetTracks()):
    if t.Type() == pcbnew.PCB_VIA_T:
        continue
    if not in_strip(t):
        continue
    nm = t.GetNetname()
    if nm not in u7nets:
        removed.append((nm, t))
        b.Remove(t)
print('removed %d strip tracks of foreign nets: %s' % (len(removed), [r[0] for r in removed]))

nmap = {}
for fp2 in b.GetFootprints():
    for p in fp2.Pads():
        nm = p.GetNetname()
        if nm:
            nmap[nm] = p.GetNetCode()
for t in list(b.GetTracks()):
    b.Remove(t)
n = 0
for net in rorder:
    if net not in nmap:
        continue
    nc = nmap[net]
    for (x1, y1, x2, y2, h, L) in segs_of[net]:
        s = pcbnew.PCB_TRACK(b)
        s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
        s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
        s.SetWidth(int(round(2 * h * 1e6)))
        s.SetLayer(LWR[int(L)])
        s.SetNetCode(nc)
        b.Add(s)
        n += 1
for net in rorder:
    if net not in nmap:
        continue
    nc = nmap[net]
    for (x, y) in vias_of[net]:
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
        v.SetWidth(int(round(rr.VIA_D * 1e6)))
        v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNetCode(nc)
        b.Add(v)
        n += 1
b.Save(DST)
print('re-routed %d nets, saved %s (%d items, %.0fs)' % (done_nets, DST, n, time.time() - t0))
for a in log[-10:]:
    print('   ', a)
