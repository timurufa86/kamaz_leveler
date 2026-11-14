import io,sys,os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
import pcbnew
IU=1e6
def check(path,label):
    d,fn=os.path.split(os.path.abspath(path))
    fp=pcbnew.FootprintLoad(d, os.path.splitext(fn)[0])
    if fp is None: print(label,'LOAD FAIL'); return
    items=[(it,it.GetLayerName()) for it in fp.GraphicalItems()]+[(fp.Reference(),fp.Reference().GetLayerName())]
    pads=[(pd.GetNumber(),pd.GetPosition(),pd.GetSize().x/2.0) for pd in fp.Pads()]
    viol=[]
    n=0
    for it,l in items:
        if 'Silk' not in l: continue
        n+=1
        w=0.0
        try: w=it.GetWidth()/IU
        except Exception: pass
        if type(it).__name__ in ('PCB_TEXT','PCB_FIELD'):
            try: w=it.GetTextThickness()/IU
            except Exception: w=0.0
        s=None
        try: s=it.GetEffectiveShape()
        except Exception: pass
        for num,pos,pr in pads:
            need=pr+w/2.0
            if s is not None:
                try: dist=s.Distance(pos)/IU
                except Exception: dist=None
            else: dist=None
            if dist is not None and dist<need:
                viol.append((round(dist-need,4), getattr(it,'GetText',lambda:'shape')(), num))
    print('%-34s silk=%-3d pads=%-3d overlaps=%d' % (label, n, len(pads), len(viol)))
    return viol
v=check('kamaz_leveler_hw.pretty/ESP32-S3-DevKitC-1_Socket_2x22_P2.54mm.kicad_mod','TARGET 2x22 (esp32s3)')
print('  full list:', sorted(v))
check('../../kicad/kamaz_leveler_hw.pretty/ESP32-DevKitC_Socket_2x19_P2.54mm.kicad_mod','BASE 2x19')
check('../../kicad/kamaz_leveler_hw.pretty/ESP32-DevKitC_Socket_2x15_P2.54mm.kicad_mod','BASE 2x15')
