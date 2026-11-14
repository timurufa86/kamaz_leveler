import sys
import os
import pcbnew
ROOT = r'C:\my\for cursor\kamaz_leveler\hardware\variants\esp32s3_ruichi158'
SRC = os.path.join(ROOT, '_work', 'GOOD_0viol.kicad_pcb')
DST = os.path.join(ROOT, '_work', '_prep.kicad_pcb')
ANT = (-13.0, 34.0, 13.0, 42.75)

b = pcbnew.LoadBoard(SRC)
moved = []
for ref, dx in (('R33', -1.6), ('R34', -1.6)):
    fp = [f for f in b.GetFootprints() if f.GetReference() == ref][0]
    p0 = fp.GetPosition()
    fp.SetPosition(pcbnew.VECTOR2I(p0.x + int(dx * 1e6), p0.y))
    moved.append((ref, p0.x / 1e6, p0.y / 1e6, p0.x / 1e6 + dx))
for m_ in moved:
    print('MOVE', m_[0], round(m_[1],3), round(m_[2],3), '->', round(m_[3],3), round(m_[2],3))

TARGETS = ['/VLV_IN3', '/VLV_IN4', '/VLV_IN5', '/VLV_IN6',
           '/ST_U3_34', '/ST_U4_34', '/BUT1', '/BUT4',
           '/ST_U3_12', '/I2C_SCL', '/I2C_SDA', '/VALVE_INFL', '+12V']
dropped = {}
for net in TARGETS:
    dropped[net] = 0
nz = 0
for t in list(b.GetTracks()):
    nm = t.GetNetname()
    if nm in dropped:
        b.Remove(t)
        dropped[nm] += 1
        continue
    if t.Type() == pcbnew.PCB_VIA_T or nm != '+3V3':
        continue
    x1, y1 = t.GetStart().x / 1e6, t.GetStart().y / 1e6
    x2, y2 = t.GetEnd().x / 1e6, t.GetEnd().y / 1e6
    h = t.GetWidth() / 2e6
    in_ant = (max(x1, x2) + h > ANT[0] and min(x1, x2) - h < ANT[2] and
              max(y1, y2) + h > ANT[1] and min(y1, y2) - h < ANT[3])
    in_win = (max(x1, x2) > -18.6 and min(x1, x2) < -14.4 and
              max(y1, y2) > 4.4 and min(y1, y2) < 10.1)
    if in_ant or in_win:
        b.Remove(t)
        nz += 1
print('dropped +3V3 tracks (antenna/moved window): %d' % nz)
print('dropped per target net: %s' % {k: v for k, v in dropped.items() if v})
b.Save(DST)
print('saved', DST)
