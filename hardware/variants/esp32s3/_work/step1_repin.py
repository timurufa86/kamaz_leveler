import io,sys,json
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
sys.stderr=open('_work/step1.err','w')
import pcbnew
MM=pcbnew.ToMM
SRC=r"kamaz_leveler_s3.kicad_pcb"
DST=r"_work/s1.kicad_pcb"
# pad -> net required by the schematic netlist (verify.net)
MAP={ "1":"+3V3_ESP","2":"+3V3_ESP","3":"unconnected-(U7-RST-Pad3)",
 "4":"/DISP_BL","5":"/DISP_CS","6":"unconnected-(U7-IO6-Pad6)","7":"unconnected-(U7-IO7-Pad7)",
 "8":"unconnected-(U7-IO15-Pad8)","9":"/DISP_DC","10":"/DISP_RES","11":"/DISP_SCLK",
 "12":"/SPEED_PULSE","13":"unconnected-(U7-IO3-Pad13)","14":"unconnected-(U7-IO46-Pad14)",
 "15":"unconnected-(U7-IO9-Pad15)","16":"unconnected-(U7-IO10-Pad16)","17":"/DISP_MOSI",
 "18":"unconnected-(U7-IO12-Pad18)","19":"/INT_MCU","20":"/VLV_EN","21":"+5V",
 "22":"GND","23":"GND","24":"unconnected-(U7-IO43-Pad24)","25":"unconnected-(U7-IO44-Pad25)",
 "26":"unconnected-(U7-IO1-Pad26)","27":"unconnected-(U7-IO2-Pad27)","28":"unconnected-(U7-IO42-Pad28)",
 "29":"unconnected-(U7-IO41-Pad29)","30":"unconnected-(U7-IO40-Pad30)","31":"unconnected-(U7-IO39-Pad31)",
 "32":"/PRES_SW","33":"unconnected-(U7-IO37-Pad33)","34":"unconnected-(U7-IO36-Pad34)",
 "35":"unconnected-(U7-IO35-Pad35)","36":"unconnected-(U7-IO0-Pad36)","37":"unconnected-(U7-IO45-Pad37)",
 "38":"unconnected-(U7-IO48-Pad38)","39":"/I2C_SCL","40":"/I2C_SDA","41":"unconnected-(U7-IO20-Pad41)",
 "42":"unconnected-(U7-IO19-Pad42)","43":"GND","44":"GND"}
TARGET12={"/DISP_BL","/DISP_CS","/DISP_DC","/DISP_RES","/DISP_SCLK","/SPEED_PULSE",
          "/DISP_MOSI","/INT_MCU","/VLV_EN","/PRES_SW","/I2C_SCL","/I2C_SDA"}
b=pcbnew.LoadBoard(SRC)
def getnet(name):
    ni=b.FindNet(name)
    if ni is None:
        ni=pcbnew.NETINFO_ITEM(b,name); b.Add(ni)
    return ni
u7=[f for f in b.GetFootprints() if f.GetReference()=="U7"][0]
changed=[]
for p in u7.Pads():
    n=p.GetNumber()
    want=MAP[n]
    if p.GetNetname()!=want:
        changed.append((n,p.GetNetname(),want))
        p.SetNet(getnet(want))
print("pad net changes:",len(changed))
for n,a,c in changed: print("  pad %2s %-32s -> %s"%(n,a,c))
# delete copper of the 12 target nets
rm=[t for t in b.GetTracks() if t.GetNetname() in TARGET12]
print("deleting tracks/vias:",len(rm),"segs:",sum(1 for t in rm if t.Type()==pcbnew.PCB_TRACE_T),
      "vias:",sum(1 for t in rm if t.Type()==pcbnew.PCB_VIA_T))
for t in rm: b.Remove(t)
# drop nets with no items
used=set()
for f in b.GetFootprints():
    for p in f.Pads(): used.add(p.GetNetname())
for t in b.GetTracks(): used.add(t.GetNetname())
for z in b.Zones(): used.add(z.GetNetname())
dead=[ni for c,ni in b.GetNetInfo().NetsByNetcode().items() if c!=0 and ni.GetNetname() not in used]
print("unused nets to remove:",len(dead),[ni.GetNetname() for ni in dead])
for ni in dead: b.Remove(ni)
b.BuildListOfNets()
nl=[ni for c,ni in b.GetNetInfo().NetsByNetcode().items() if c!=0]
print("nets after:",len(nl))
tr=list(b.GetTracks())
print("segs",sum(1 for t in tr if t.Type()==pcbnew.PCB_TRACE_T),"vias",sum(1 for t in tr if t.Type()==pcbnew.PCB_VIA_T))
print("fp",len(list(b.GetFootprints())),"pads",sum(f.GetPadCount() for f in b.GetFootprints()),
      "zones",b.GetAreaCount())
pcbnew.SaveBoard(DST,b)
print("saved",DST)
