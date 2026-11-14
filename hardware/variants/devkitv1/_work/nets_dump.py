import sys, io, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
nets = collections.defaultdict(list)
for fp in b.GetFootprints():
    for pad in fp.Pads():
        nc = pad.GetNetCode()
        if nc:
            nets[pad.GetNetname()].append('%s.%s' % (fp.GetReference(), pad.GetNumber()))
print('board nets with pads:', len(nets))
singles = {k:v for k,v in nets.items() if len(v)==1}
print('single-pad nets: %d' % len(singles))
for k,v in sorted(singles.items()): print('   SINGLE', k, v)
print('--- nets mentioning U7 or EN:')
for k,v in sorted(nets.items()):
    if 'U7' in k or 'EN' in k.upper():
        print('   ', k, v)
