import sys, re
sys.stdout.reconfigure(encoding='utf-8', errors='replace')
PCB = "kamaz_leveler_ruichi158.kicad_pcb"
src = open(PCB, encoding='utf-8').read().split('\n')

DROP_HEAD = {'segment','via','zone'}
EDGE_HEADS = {'gr_line','gr_arc','gr_circle','gr_poly','gr_rect','gr_curve'}
HOLES = {'H1','H2','H3','H4'}

out = []
i = 0
kept = dropped = 0
n = len(src)
while i < n:
    line = src[i]
    if line.startswith('\t(') and not line.startswith('\t\t'):
        head = re.match(r'\t\(([a-z_0-9]+)', line).group(1)
        # collect whole block by paren depth
        depth = 0; blk = []
        j = i
        while j < n:
            depth += src[j].count('(') - src[j].count(')')
            blk.append(src[j]); j += 1
            if depth == 0: break
        text = '\n'.join(blk)
        drop = False
        if head in DROP_HEAD: drop = True
        elif head in EDGE_HEADS and '(layer "Edge.Cuts")' in text: drop = True
        elif head == 'footprint':
            m = re.search(r'\(property "Reference" "([^"]+)"', text)
            if m and m.group(1) in HOLES: drop = True
        if drop: dropped += 1
        else: out.extend(blk); kept += 1
        i = j
    else:
        out.append(line); i += 1

open(PCB, 'w', encoding='utf-8', newline='\n').write('\n'.join(out))
print('blocks kept', kept, 'dropped', dropped)
