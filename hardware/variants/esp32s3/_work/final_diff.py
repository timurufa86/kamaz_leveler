import io, sys, re, hashlib, difflib
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def sha(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest()

cur = 'kamaz_leveler_s3.kicad_sch'
bak = '_work/BACKUP_sch_before_repin.kicad_sch'
a = open(bak, encoding='utf-8').read()
b = open(cur, encoding='utf-8').read()

print("sha256 before:", sha(bak))
print("sha256 after :", sha(cur))
print()

al = a.split('\n')
bl = b.split('\n')
diff = list(difflib.unified_diff(al, bl, lineterm='', n=0))
adds = [l for l in diff if l.startswith('+') and not l.startswith('+++')]
dels = [l for l in diff if l.startswith('-') and not l.startswith('---')]
print("lines before/after: %d / %d" % (len(al), len(bl)))
print("diff: %d added, %d deleted" % (len(adds), len(dels)))
print("\n--- deleted lines ---")
for l in dels:
    print("  " + l)
print("\n--- added lines ---")
for l in adds:
    print("  " + l)

def labels(t):
    return re.findall(r'\(label\s+"([^"]+)"', t)
def glabels(t):
    return re.findall(r'\(global_label\s+"([^"]+)"', t)
for f, n in (('label', labels), ('global_label', glabels)):
    la, lb = n(a), n(b)
    print("\n%s count before/after: %d / %d   multiset equal: %s" % (f, len(la), len(lb), sorted(la) == sorted(lb)))
    if sorted(la) != sorted(lb):
        import collections
        ca, cb = collections.Counter(la), collections.Counter(lb)
        print("   only_before:", {k: v for k, v in (ca - cb).items()})
        print("   only_after :", {k: v for k, v in (cb - ca).items()})

print("\nno_connect count before/after: %d / %d" % (a.count('(no_connect'), b.count('(no_connect')))
print("symbol count before/after: %d / %d" % (a.count('(lib_id'), b.count('(lib_id')))
print("wire count before/after: %d / %d" % (a.count('(wire\n'), b.count('(wire\n')))
print("uuid count before/after: %d / %d" % (a.count('(uuid'), b.count('(uuid')))
