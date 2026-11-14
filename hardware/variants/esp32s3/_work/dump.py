import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
fps = list(b.GetFootprints())
tracks = list(b.GetTracks())
print('footprints', len(fps), 'track/via items', len(tracks), 'zones', b.GetAreaCount(), 'drawings', len(list(b.GetDrawings())))
print('nets', b.GetNetCount())
u7 = [f for f in fps if f.GetReference() == 'U7'][0]
print('U7 fpid', u7.GetFPID().GetLibItemName(), 'pos', u7.GetPosition().x/1e6, u7.GetPosition().y/1e6, 'orient', u7.GetOrientationDegrees())
print('U7 value', u7.GetValue(), 'pads', len(list(u7.Pads())))
for pad in sorted(u7.Pads(), key=lambda p: int(p.GetNumber())):
    p = pad.GetPosition()
    print('  pad %-3s net=%-34r pos %.3f,%.3f type=%s' % (pad.GetNumber(), pad.GetNetname(), p.x/1e6, p.y/1e6, pad.GetAttribute()))
