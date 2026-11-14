import io
import sys
import json
import math
import os
import time
import numpy as np

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
WORK = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\_work'
sys.path.insert(0, WORK)
import pcbnew
import rr
from rr import Model, NX, NY, CELL, ix, iy, cx, cy, VIA_R

SRC = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158\kamaz_leveler_ruichi158.kicad_pcb'


def width_of(net):
    if net in ('/VALVE_BUB1', '/VALVE_BUB2', '/VALVE_BUB3', '/VALVE_BUB4', '/VALVE_DEFL',
               '/VALVE_INFL') or net.startswith('/VLV_IN'):
        return 0.8
    if net in ('+24V', '/+24V_IN', '/VIN_FUSED'):
        return 0.8
    if net == '+12V':
        return 0.6
    if net == '+5V':
        return 0.5
    if net == '+3V3':
        return 0.4
    return 0.15


def width_candidates(net):
    w = width_of(net)
    out = [w]
    if w > 0.15:
        for c in (0.5, 0.4, 0.3, 0.2, 0.15):
            if c < w - 1e-9:
                out.append(c)
    return out


PH_A = ['/INT_MCU', 'GND', '+3V3']
PH_B = ['/VLV_IN1', '/VLV_IN2', '/VLV_IN3', '/VLV_IN4', '/VLV_IN5', '/VLV_IN6',
        '/ST_U3_12', '/ST_U3_34', '/ST_U4_12', '/ST_U4_34',
        '/BUT1', '/BUT2', '/BUT3', '/BUT4', '/BUT5']
CLUSTER = ('U1', 'U2', 'U3', 'U4', 'U5', 'U6', 'U8', 'U11', 'U12')
BOOST = []

PH_C = ['/VALVE_BUB1', '/VALVE_BUB2', '/VALVE_BUB3', '/VALVE_BUB4', '/VALVE_DEFL', '/VALVE_INFL',
        '+24V', '/+24V_IN', '/VIN_FUSED', '/VIN_FILT']
PH_D = ['+12V', '+5V']
PH_E = ['/DISP_BL', '/DISP_CS', '/DISP_DC', '/DISP_RES', '/DISP_SCLK', '/DISP_MOSI',
        '/I2C_SCL', '/I2C_SDA', '/PRES_SW', '/VLV_EN', '/SPEED_PULSE', '/SPEED_SIG', '/SP_DIV']
SKIP = ()


def plan(m):
    multi = {}
    for p in m.pads:
        multi.setdefault(p['net'], []).append(p)
    nets = [n for n, ps in multi.items() if len(ps) > 1 and n not in SKIP and n != '__none__']

    def diag(n):
        ps = multi[n]
        xs = [p['x'] for p in ps]
        ys = [p['y'] for p in ps]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    order = []
    for n in PH_A:
        if n in nets:
            order.append(n)
    for n in BOOST:
        n2 = n if n in nets else ('/' + n if '/' + n in nets else None)
        if n2 and n2 not in order:
            order.append(n2)
    for n in PH_C:
        if n in nets and n not in order:
            order.append(n)
    # nets attached to the fine-pitch cluster first (bbox order)
    attach = set()
    for p in m.pads:
        if p['ref'] in CLUSTER:
            attach.add(p['net'])
    cl = [n for n in nets if n in attach and n not in order]
    cl.sort(key=lambda n: diag(n))
    order += cl
    for n in PH_D:
        if n in nets and n not in order:
            order.append(n)
    rest = [n for n in nets if n not in order]
    rest.sort(key=lambda n: diag(n))
    return order + rest


def sd_pad(pad, x, y):
    u = pad['c'] * (x - pad['x']) + pad['s'] * (y - pad['y'])
    v = -pad['s'] * (x - pad['x']) + pad['c'] * (y - pad['y'])
    qx = abs(u) - (pad['hx'] - pad['r'])
    qy = abs(v) - (pad['hy'] - pad['r'])
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - pad['r']


def trim_to_pad(pad, x0, y0, x1, y1, depth=0.12):
    lo, hi = 0.0, 1.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if sd_pad(pad, x0 + (x1 - x0) * mid, y0 + (y1 - y0) * mid) < -depth:
            lo = mid
        else:
            hi = mid
    return (x0 + (x1 - x0) * lo, y0 + (y1 - y0) * lo)


def access_cells(m, pad, hw, L, blk, want_ring=1.1):
    ext = max(pad['hx'], pad['hy'])
    base = ext + rr.CLR + hw + 0.06
    out = []
    for k in range(0, 16):
        r = base + 0.07 * k
        if r > base + want_ring:
            break
        for t in range(24):
            a = 2 * math.pi * t / 24.0
            x = pad['x'] + r * math.cos(a)
            y = pad['y'] + r * math.sin(a)
            i, j = ix(x), iy(y)
            if not (0 <= i < NX and 0 <= j < NY):
                continue
            if blk[j, i]:
                continue
            n = max(2, int(r / 0.04) + 1)
            ok = True
            for s in range(n + 1):
                f = s / n
                if blk[iy(pad['y'] + (y - pad['y']) * f), ix(pad['x'] + (x - pad['x']) * f)]:
                    ok = False
                    break
            if ok:
                out.append((x, y))
    seen = set()
    res = []
    for (x, y) in out:
        k = (ix(x), iy(y))
        if k in seen:
            continue
        seen.add(k)
        res.append((x, y))
    return res


def free_near(blk, x, y, Rs=(0.0, 0.1, 0.2, 0.35, 0.5, 0.7, 1.0)):
    for r in Rs:
        cand = [(x, y)]
        if r > 0:
            cand = [(x + r * math.cos(2 * math.pi * t / 12), y + r * math.sin(2 * math.pi * t / 12))
                    for t in range(12)]
        for (px, py) in cand:
            i, j = ix(px), iy(py)
            if 0 <= i < NX and 0 <= j < NY and not blk[j, i]:
                return (px, py)
    return None


def tree_seeds(segs, vias, step=0.45):
    seeds = []
    for (x1, y1, x2, y2, h, L) in segs:
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / step))
        for t in range(n + 1):
            f = t / n
            seeds.append((x1 + (x2 - x1) * f, y1 + (y2 - y1) * f, L, 0.30))
    for (vx, vy) in vias:
        seeds.append((vx, vy, 0, 0.0))
        seeds.append((vx, vy, 1, 0.0))
    return seeds


def project_to_tree(px, py, segs, Lwant):
    best = None
    for (x1, y1, x2, y2, h, L) in segs:
        if L != Lwant:
            continue
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        qx, qy = x1 + t * dx, y1 + t * dy
        d = math.hypot(px - qx, py - qy)
        if best is None or d < best[0]:
            best = (d, qx, qy, L)
    if best is None:
        return (px, py, Lwant)
    return (best[1], best[2], best[3])


def ownmask_add(own, segs, vias):
    for (x1, y1, x2, y2, h, L) in segs:
        i0, i1 = max(0, ix(min(x1, x2) - h - 0.25)), min(NX - 1, ix(max(x1, x2) + h + 0.25))
        j0, j1 = max(0, iy(min(y1, y2) - h - 0.25)), min(NY - 1, iy(max(y1, y2) + h + 0.25))
        X = (rr.X0 + np.arange(i0, i1 + 1) * CELL)[None, :]
        Y = (rr.Y0 + np.arange(j0, j1 + 1) * CELL)[:, None]
        own[j0:j1 + 1, i0:i1 + 1] |= (rr.dseg(X, Y, x1, y1, x2, y2) <= h)
    for (vx, vy) in vias:
        i, j = ix(vx), iy(vy)
        own[max(0, j - 3):j + 4, max(0, i - 3):i + 4] = True


def weiszfeld(pts, iters=80):
    x = sum(p[0] for p in pts) / len(pts)
    y = sum(p[1] for p in pts) / len(pts)
    for _ in range(iters):
        num = [0.0, 0.0]
        den = 0.0
        for (px, py) in pts:
            d = max(1e-6, math.hypot(px - x, py - y))
            w = 1.0 / d
            num[0] += px * w
            num[1] += py * w
            den += w
        nx_, ny_ = num[0] / den, num[1] / den
        if math.hypot(nx_ - x, ny_ - y) < 1e-6:
            return (nx_, ny_)
        x, y = nx_, ny_
    return (x, y)


def repair_endpoints(m, segs, vias, pads, netname):
    """connect free segment ends that stop near (but not on) own copper"""
    n = len(segs)
    for a in range(n):
        x1, y1, x2, y2, h, L = segs[a]
        for which in (0, 1):
            ex, ey = (x1, y1) if which == 0 else (x2, y2)
            if any(sd_pad(p, ex, ey) <= -0.05 for p in pads):
                continue
            if any(math.hypot(ex - vx, ey - vy) < 1e-6 for (vx, vy) in vias):
                continue
            touched = False
            best = None
            for b in range(n):
                if b == a:
                    continue
                bx1, by1, bx2, by2, bh, bL = segs[b]
                if bL != L:
                    continue
                dx, dy = bx2 - bx1, by2 - by1
                d2 = dx * dx + dy * dy
                t = 0.0 if d2 < 1e-12 else max(0.0, min(1.0, ((ex - bx1) * dx + (ey - by1) * dy) / d2))
                qx, qy = bx1 + t * dx, by1 + t * dy
                d = math.hypot(ex - qx, ey - qy)
                if d < 0.01:
                    touched = True
                    break
                if d < bh + 0.05 and (best is None or d < best[0]):
                    best = (d, qx, qy)
            if touched:
                continue
            if best is None:
                # nothing within copper width: look further (hub stub trimming)
                for b in range(n):
                    if b == a:
                        continue
                    bx1, by1, bx2, by2, bh, bL = segs[b]
                    if bL != L:
                        continue
                    dx, dy = bx2 - bx1, by2 - by1
                    d2 = dx * dx + dy * dy
                    t = 0.0 if d2 < 1e-12 else max(0.0, min(1.0, ((ex - bx1) * dx + (ey - by1) * dy) / d2))
                    qx, qy = bx1 + t * dx, by1 + t * dy
                    d = math.hypot(ex - qx, ey - qy)
                    if d < 4.0 and (best is None or d < best[0]):
                        best = (d, qx, qy)
            if best is None:
                continue
            nx1, ny1 = (best[1], best[2]) if which == 0 else (x1, y1)
            nx2, ny2 = (x2, y2) if which == 0 else (best[1], best[2])
            if not m.seg_clear(nx1, ny1, nx2, ny2, h, L, netname):
                continue
            if which == 0:
                segs[a] = (best[1], best[2], x2, y2, h, L)
                x1, y1 = best[1], best[2]
            else:
                segs[a] = (x1, y1, best[1], best[2], h, L)
                x2, y2 = best[1], best[2]


def route_net(m, netname, hw, log, margin=4.0, via_cost=1.0, use_hub=True):
    pads = m.netpads(netname)
    # drop pads whose centre lies inside another pad of the same footprint
    # (U1 EP thermal vias sit inside the exposed pad)
    keep = []
    for p in pads:
        inside = False
        for q in m.pads:
            if q is p or q['ref'] != p['ref']:
                continue
            if sd_pad(q, p['x'], p['y']) < -0.05:
                inside = True
                break
        if not inside:
            keep.append(p)
    pads = keep
    if len(pads) < 2:
        return [], []
    pens = m.pen_for_net(netname)
    blkF, blkB, blkV = m.pen_masks(pens, hw)
    blk = [blkF, blkB]
    own = np.zeros((NY, NX), bool)
    segs = []
    vias = []
    start = max(pads, key=lambda p: p['hx'] * p['hy'])
    rest = [p for p in pads if p is not start]
    targets = []
    hub = None
    if use_hub and 3 <= len(pads) <= 4:
        hub = weiszfeld([(p['x'], p['y']) for p in pads])
        targets.append(('hub', hub, None))
    for p in rest:
        targets.append(('pad', (p['x'], p['y']), p))
    prev = None
    for (kind, pt, pad) in targets:
        if not (segs or vias):
            Ls = [0, 1] if (start['onF'] and start['onB']) else ([0] if start['onF'] else [1])
            seeds = []
            for L in Ls:
                for (x, y) in access_cells(m, start, hw, L, blk[L]):
                    seeds.append((x, y, L, math.hypot(x - start['x'], y - start['y'])))
            if not seeds:
                log.append('%s: no access from %s.%s' % (netname, start['ref'], start['num']))
                return None, None
        elif prev == 'hub':
            seeds = [(hub[0], hub[1], m._last_end_layer, 0.0)]
        else:
            seeds = tree_seeds(segs, vias)
        if kind == 'hub':
            goalset = []
            gc = pt
            for L in (0, 1):
                q = free_near(blk[L], pt[0], pt[1])
                if q:
                    for (x, y) in cells_near(blk[L], q, 0.35):
                        goalset.append((L, ix(x), iy(y)))
            if not goalset:
                log.append('%s: hub unreachable' % netname)
                return None, None
        else:
            gc = (pad['x'], pad['y'])
            Ls = [0, 1] if (pad['onF'] and pad['onB']) else ([0] if pad['onF'] else [1])
            goalset = []
            for L in Ls:
                for (x, y) in access_cells(m, pad, hw, L, blk[L]):
                    goalset.append((L, ix(x), iy(y)))
            if not goalset:
                log.append('%s: no access to %s.%s' % (netname, pad['ref'], pad['num']))
                return None, None
        pts = m.astar(seeds, goalset, gc, hw, pens, margin=margin, ownpen=own, via_cost=via_cost)
        if pts is None:
            pts = m.astar(seeds, goalset, gc, hw, pens, margin=90.0, ownpen=own,
                          maxexp=1500000, via_cost=via_cost)
        if pts is None:
            log.append('%s: SKIP %s (%.2f,%.2f)' % (netname, kind, gc[0], gc[1]))
            continue
        if (segs or vias) and prev != 'hub':
            (qx, qy, ql) = project_to_tree(pts[0][0], pts[0][1], segs, pts[0][2])
            if m.seg_clear(qx, qy, pts[1][0], pts[1][1], hw, ql, netname) if len(pts) > 1 else True:
                pts[0] = (qx, qy, ql, pts[0][3])
        runs = []
        cur = []
        for (x, y, L, via) in pts:
            if cur and cur[-1][2] != L:
                runs.append(cur)
                cur = []
            cur.append((x, y, L))
        if cur:
            runs.append(cur)
        if not (segs or vias):
            runs[0].insert(0, (start['x'], start['y'], runs[0][0][2]))
        if kind == 'pad':
            runs[-1].append((pad['x'], pad['y'], runs[-1][-1][2]))
        for k, run in enumerate(runs):
            pl = rr.collinear_merge([(p[0], p[1]) for p in run])
            pl = rr.smooth(pl, m, hw, run[0][2], netname)
            if k == 0 and len(pl) > 1 and not (segs or vias):
                pl[0] = trim_to_pad(start, pl[0][0], pl[0][1], pl[1][0], pl[1][1])
            if k == len(runs) - 1 and len(pl) > 1 and kind == 'pad':
                pl[-1] = trim_to_pad(pad, pl[-1][0], pl[-1][1], pl[-2][0], pl[-2][1])
            for t in range(len(pl) - 1):
                (ax, ay), (bx, by) = pl[t], pl[t + 1]
                if math.hypot(bx - ax, by - ay) < 1e-4:
                    continue
                segs.append((ax, ay, bx, by, hw, run[0][2]))
            if k + 1 < len(runs):
                vias.append((run[-1][0], run[-1][1]))
        m._last_end_layer = runs[-1][-1][2]
        if kind == 'hub':
            hub = (runs[-1][-1][0], runs[-1][-1][1])
        ownmask_add(own, segs, vias)
        prev = kind
    return segs, vias


def cells_near(blk, pt, R):
    out = []
    for k in range(4):
        r = R * k / 3.0
        for t in range(8):
            a = 2 * math.pi * t / 8.0
            x = pt[0] + r * math.cos(a)
            y = pt[1] + r * math.sin(a)
            i, j = ix(x), iy(y)
            if 0 <= i < NX and 0 <= j < NY and not blk[j, i]:
                out.append((x, y))
    return out


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    out = sys.argv[2] if len(sys.argv) > 2 else r'routed.kicad_pcb'
    t0 = time.time()
    m = Model(SRC)
    m._last_end_layer = 0
    m.build()
    print('model: pads %d nets %d  (%.1fs)' % (len(m.pads), len(m.nets), time.time() - t0))
    if len(sys.argv) > 3 and sys.argv[3]:
        BOOST.extend(x.strip().lstrip('/') for x in sys.argv[3].split(','))
    order = plan(m)
    if only and only != 'all':
        keep = [x.strip().lstrip('/') for x in only.split(',')]
        order = [n for n in order if n.lstrip('/') in keep]
    log = []
    total = []
    t1 = time.time()
    failed = []
    for net in order:
        ts = time.time()
        got = None
        for w in width_candidates(net):
            segs, vias = route_net(m, net, w / 2.0, log)
            if segs:
                got = (w, segs, vias)
                break
        if got is None:
            print('%-14s FAIL' % net, flush=True)
            failed.append(net)
            continue
        w, segs, vias = got
        L = sum(math.hypot(s[2] - s[0], s[3] - s[1]) for s in segs)
        print('%-14s w%.2f segs %3d vias %2d len %6.2f  (%.1fs)' %
              (net, w, len(segs), len(vias), L, time.time() - ts), flush=True)
        m.add_copper(segs, vias)
        total.append((net, segs, vias))
    wused = {net: round(2 * segs[0][4], 3) for (net, segs, vias) in total}
    if failed:
        print('--- rip-up pass for %d nets' % len(failed), flush=True)
        segs_of = {net: (segs, vias) for (net, segs, vias) in total}
        rorder = [net for (net, segs, vias) in total]
        for f in list(failed):
            fp = m.netpads(f)
            fx0 = min(p['x'] for p in fp) - 4.0
            fx1 = max(p['x'] for p in fp) + 4.0
            fy0 = min(p['y'] for p in fp) - 4.0
            fy1 = max(p['y'] for p in fp) + 4.0
            cen = (sum(p['x'] for p in fp) / len(fp), sum(p['y'] for p in fp) / len(fp))

            def prox(n):
                ss = segs_of[n][0]
                xx = [(s[0] + s[2]) / 2 for s in ss]
                yy = [(s[1] + s[3]) / 2 for s in ss]
                return min(math.hypot(a - cen[0], b - cen[1]) for a, b in zip(xx, yy))

            cand = []
            for n in rorder:
                hit = False
                for (x1, y1, x2, y2, h, L) in segs_of[n][0]:
                    if (max(x1, x2) + h > fx0 and min(x1, x2) - h < fx1 and
                            max(y1, y2) + h > fy0 and min(y1, y2) - h < fy1):
                        hit = True
                        break
                if hit:
                    cand.append(n)
            cand.sort(key=prox)
            done = False
            for k in (1, 2, 3, 5, 8, 12, 20):
                rip = set(cand[:k])
                m.reset_pen()
                for n in rorder:
                    if n in rip or n == f:
                        continue
                    m.add_copper(segs_of[n][0], segs_of[n][1])
                segs_f, vias_f = route_net(m, f, min(width_of(f), 0.3) / 2.0, log,
                                           margin=10.0, via_cost=1.2, use_hub=False)
                if segs_f is None:
                    continue
                m.add_copper(segs_f, vias_f)
                ok2 = True
                newgeo = {}
                for n in [x for x in rorder if x in rip]:
                    s2, v2 = route_net(m, n, wused.get(n, 0.15) / 2.0, log,
                                       margin=10.0, via_cost=2.0)
                    if s2 is None:
                        ok2 = False
                        break
                    m.add_copper(s2, v2)
                    newgeo[n] = (s2, v2)
                if not ok2:
                    continue
                for n, g in newgeo.items():
                    segs_of[n] = g
                segs_of[f] = (segs_f, vias_f)
                print('%-14s RIPUP ok k=%d  segs %d vias %d' %
                      (f, k, len(segs_f), len(vias_f)), flush=True)
                done = True
                break
            if done:
                failed.remove(f)
        total = [(n, segs_of[n][0], segs_of[n][1]) for n in rorder if n in segs_of]
        for f in failed:
            print('%-14s UNRESOLVED' % f, flush=True)
    if False:
        still = []
        for net in failed:
            got = None
            for w in (min(width_of(net), 0.2), 0.15):
                for hub in (True, False):
                    for vc in (1.2,):
                        log2 = []
                        segs, vias = route_net(m, net, w / 2.0, log2, margin=12.0,
                                               via_cost=vc, use_hub=hub)
                        if segs:
                            got = (w, segs, vias)
                            break
                    if got:
                        break
                if got:
                    break
            if got is None:
                still.append(net)
                print('%-14s STILL FAIL' % net, flush=True)
                continue
            w, segs, vias = got
            m.add_copper(segs, vias)
            total.append((net, segs, vias))
            print('%-14s RETRY w%.2f segs %3d vias %2d' % (net, w, len(segs), len(vias)), flush=True)
        failed = still
    print('--- routed %d/%d in %.1fs, failures: %d' % (len(total), len(order), time.time() - t1, len(failed)))
    nfix = 0
    for (net, segs, vias) in total:
        before = [tuple(x) for x in segs]
        repair_endpoints(m, segs, vias, m.netpads(net), net)
        if [tuple(x) for x in segs] != before:
            nfix += 1
    print('endpoint repair touched %d nets' % nfix)
    for a in log:
        print('   ', a)
    b = m.b
    nmap = {}
    for fp in b.GetFootprints():
        for p in fp.Pads():
            nm = p.GetNetname()
            if nm:
                nmap[nm] = p.GetNetCode()
    LAY = {0: pcbnew.F_Cu, 1: pcbnew.B_Cu}
    nseg = nvia = 0
    for (net, segs, vias) in total:
        nc = nmap[net]
        for (x1, y1, x2, y2, h, L) in segs:
            s = pcbnew.PCB_TRACK(b)
            s.SetStart(pcbnew.VECTOR2I(int(round(x1 * 1e6)), int(round(y1 * 1e6))))
            s.SetEnd(pcbnew.VECTOR2I(int(round(x2 * 1e6)), int(round(y2 * 1e6))))
            s.SetWidth(int(round(2 * h * 1e6)))
            s.SetLayer(LAY[L])
            s.SetNetCode(nc)
            b.Add(s)
            nseg += 1
        for (x, y) in vias:
            v = pcbnew.PCB_VIA(b)
            v.SetPosition(pcbnew.VECTOR2I(int(round(x * 1e6)), int(round(y * 1e6))))
            v.SetWidth(int(round(rr.VIA_D * 1e6)))
            v.SetDrill(int(round(rr.VIA_DRILL * 1e6)))
            v.SetViaType(pcbnew.VIATYPE_THROUGH)
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNetCode(nc)
            b.Add(v)
            nvia += 1
    p = os.path.join(r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158', out)
    b.Save(p)
    print('saved %s : %d tracks, %d vias' % (p, nseg, nvia))
    json.dump([(n, [[float(v) for v in s] for s in segs], [[float(v) for v in q] for q in vias])
               for (n, segs, vias) in total],
              open(os.path.join(WORK, 'routes.json'), 'w'))


if __name__ == '__main__':
    main()
