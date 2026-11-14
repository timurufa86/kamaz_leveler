import sys, io, collections
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)
import pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
trs = [t for t in b.GetTracks() if t.GetClass()=='PCB_TRACK']
def pt(v): return (v.x, v.y)
inc = collections.defaultdict(list)
for i,t in enumerate(trs):
    k=(t.GetNetCode(), t.GetLayer())
    inc[(k, pt(t.GetStart()))].append(i)
    inc[(k, pt(t.GetEnd()))].append(i)
via_pts = set(pt(t.GetPosition()) for t in b.GetTracks() if t.GetClass()=='PCB_VIA')
ends = [(pt(t.GetStart()), pt(t.GetEnd())) for t in trs]
def cross(a,bb): return a[0]*bb[1]-a[1]*bb[0]
c1=c2=c3=c4=c5=0; ok=0
for (k,p),lst in inc.items():
    if len(lst)!=2: c1+=1; continue
    if p in via_pts: c2+=1; continue
    i,j=lst
    if i==j: c1+=1; continue
    if trs[i].GetWidth()!=trs[j].GetWidth(): c3+=1; continue
    a0,a1=ends[i]; b0,b1=ends[j]
    A = a1 if a0==p else a0
    B = b1 if b0==p else b0
    v1=(A[0]-p[0],A[1]-p[1]); v2=(B[0]-p[0],B[1]-p[1])
    if cross(v1,v2)!=0: c4+=1; continue
    if v1[0]*v2[0]+v1[1]*v2[1]>=0: c5+=1; continue
    ok+=1
print('rejected: incidence!=2 %d, via %d, width %d, noncollinear %d, uturn %d | MERGEABLE %d' % (c1,c2,c3,c4,c5,ok))
