import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
tr = list(b.GetTracks())
segs = [t for t in tr if t.Type() == pcbnew.PCB_TRACE_T]
vias = [t for t in tr if t.Type() == pcbnew.PCB_VIA_T]
print('%-22s footprints=%d tracks=%d vias=%d zones=%d pads=%d nets=%d drawings=%d'
      % (sys.argv[2], len(list(b.GetFootprints())), len(segs), len(vias), len(list(b.Zones())),
         sum(len(list(f.Pads())) for f in b.GetFootprints()), b.GetNetCount(), len(list(b.GetDrawings()))))
print('   board bbox:', b.GetBoardEdgesBoundingBox().GetWidth()/1e6, 'x', b.GetBoardEdgesBoundingBox().GetHeight()/1e6, 'mm')
refs = sorted(f.GetReference() for f in b.GetFootprints())
print('   R43 present:', 'R43' in refs)
print('   first/last refs:', refs[0], refs[-1], 'count', len(refs))
