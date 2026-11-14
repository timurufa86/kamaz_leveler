import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
want = set(sys.argv[2].split(','))
for fp in b.GetFootprints():
    if fp.GetReference() in want:
        print('FP', fp.GetReference())
        try:
            for f in fp.GetFields():
                print('   field %-12s = %r' % (f.GetName(), f.GetText()))
        except Exception as e:
            print('   GetFields err', e)
