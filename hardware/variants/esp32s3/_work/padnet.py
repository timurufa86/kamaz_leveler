import io,sys,re
sys.stdout=io.TextIOWrapper(sys.stdout.buffer,encoding='utf-8',errors='replace')
txt=open('_work/verify.net',encoding='utf-8').read()
start=txt.index("\n\t(nets")
body=txt[start:]
hdr=[(m.start(),m.group(1),m.group(2)) for m in re.finditer(r'\(net\s*\n\s*\(code "(\d+)"\)\s*\n\s*\(name "([^"]*)"\)',body)]
hdr.append((len(body),None,None))
u7={}; nets=0
for k in range(len(hdr)-1):
    s,code,name=hdr[k]; blk=body[s:hdr[k+1][0]]; nets+=1
    for node in re.finditer(r'\(ref "([^"]+)"\)\s*\n\s*\(pin "([^"]+)"\)(?:\s*\n\s*\(pinfunction "([^"]*)"\))?(?:\s*\n\s*\(pintype "([^"]*)"\))?',blk):
        if node.group(1)=='U7': u7[node.group(2)]=(name,node.group(3))
print("nets in netlist:",nets)
for n in sorted(u7,key=lambda x:int(x)):
    print("  pad %2s -> %-32s (%s)"%(n,u7[n][0],u7[n][1]))
missing=[str(i) for i in range(1,45) if str(i) not in u7]
print("pads without net:",missing)
