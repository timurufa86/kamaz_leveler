import io,sys,json
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/terms.err','w')
import pcbnew
MM=pcbnew.ToMM
b=pcbnew.LoadBoard(r"kamaz_leveler_s3.kicad_pcb")
NETS=["/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE","/DISP_MOSI",
      "/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"]
out={}
for f in b.GetFootprints():
    for p in f.Pads():
        nm=p.GetNetname()
        if nm in NETS:
            out.setdefault(nm,[]).append({
              'ref':f.GetReference(),'pad':p.GetNumber(),
              'x':round(MM(p.GetPosition().x),3),'y':round(MM(p.GetPosition().y),3),
              'szx':round(MM(p.GetSize().x),3),'szy':round(MM(p.GetSize().y),3),
              'ang':round(p.GetOrientationDegrees()%180,2),'pth':p.GetAttribute()==pcbnew.PAD_ATTRIB_PTH,
              'shape':p.GetShape(),'drill':round(MM(p.GetDrillSize().x),3) if p.GetDrillSize().x>0 else 0})
for nm in NETS:
    ps=out.get(nm,[])
    print(f"{nm}: {len(ps)} pads")
    for p in ps:
        print("   %-6s.%-3s (%8.3f;%7.3f) %.2fx%.2f ang=%5.1f %s drill=%.2f shape=%d"%(
          p['ref'],p['pad'],p['x'],p['y'],p['szx'],p['szy'],p['ang'],'PTH' if p['pth'] else 'SMD',p['drill'],p['shape']))
json.dump(out,open('_work/terms.json','w'),indent=1)
