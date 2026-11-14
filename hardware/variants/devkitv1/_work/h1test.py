import sys, io, re, shutil
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace', line_buffering=True)

src = 'kamaz_leveler_devkitv1.kicad_sch'
dst = '_work/h1test.kicad_sch'
text = open(src, encoding='utf-8').read()
shutil.copy(src, dst)

# remove no_connect blocks at pins 1 (72.39 251.46) and 3 (72.39 256.54)
for target in ('72.39 251.46', '72.39 256.54'):
    pat = '\t(no_connect\n\t\t(at ' + target + '\n\t\t)\n\t\t(uuid "[^"]+"\n\t\t)\n\t)\n'
    new, n = re.subn(pat, '', text)
    print('remove no_connect at', target, '->', n)
    text = new
open(dst, 'w', encoding='utf-8', newline='').write(text)
print('written', dst, len(text))
