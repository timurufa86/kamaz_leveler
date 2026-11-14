import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
fps = list(b.GetFootprints())
print('footprint count', len(fps))
refs = sorted(f.GetReference() for f in fps)
print('refs sample', refs[:5], '...', refs[-5:])
for fp in fps:
    if fp.GetReference() == sys.argv[2]:
        print('FP', fp.GetReference(), fp.GetFPID().GetLibItemName(), fp.GetPosition(), fp.GetOrientationDegrees())
        pads = sorted(fp.Pads(), key=lambda p: int(p.GetNumber()) if p.GetNumber().isdigit() else 0)
        for pad in pads:
            print('PAD %-3s net=%r netcode=%s' % (pad.GetNumber(), pad.GetNetname(), pad.GetNetCode()))
