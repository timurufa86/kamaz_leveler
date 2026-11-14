import io,sys,json
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/lay.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
d=json.load(open('_work/terms.json'))
for nm,ps in d.items():
    for p in ps:
        fp=[f for f in b.GetFootprints() if f.GetReference()==p['ref']][0]
        pad=[q for q in fp.Pads() if q.GetNumber()==p['pad']][0]
        lay=[b.GetLayerName(l) for l in pad.GetLayerSet().Seq()]
        d[nm]=d[nm]
        p['layers']=lay
for nm,ps in d.items():
    print(nm)
    for p in ps: print("   %-6s.%-3s (%8.3f;%7.3f) %s"%(p['ref'],p['pad'],p['x'],p['y'],','.join(p['layers'])))
json.dump(d,open('_work/terms.json','w'),indent=1)
print()
for i,z in enumerate(b.Zones()):
    print("zone%d %s island_mode=%s thr=%s pad_conn=%s thermal=%s/%s"%(
        i,z.GetNetname(),z.GetIslandRemovalMode(),MM(z.GetMinIslandArea()),z.GetPadConnection(),
        MM(z.GetThermalReliefGap()),MM(z.GetThermalReliefSpokeWidth())))
