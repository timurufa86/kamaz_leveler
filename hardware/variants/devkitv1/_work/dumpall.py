import sys, io, json
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
out = {}
for fp in b.GetFootprints():
    r = fp.GetReference()
    for pad in fp.Pads():
        out['%s.%s' % (r, pad.GetNumber())] = pad.GetNetname()
json.dump(out, open(sys.argv[2], 'w', encoding='utf-8'), indent=0, sort_keys=True, ensure_ascii=False)
print('pads dumped:', len(out), '->', sys.argv[2])
print('nets on board:', b.GetNetCount())
