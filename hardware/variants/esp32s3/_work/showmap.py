import sys
rows = open(sys.argv[1], encoding='utf-8').read().splitlines()
step = int(sys.argv[2]) if len(sys.argv) > 2 else 1
for k, r in enumerate(rows):
    if k == 0:
        print(r); continue
    ys = r[:7]; body = r[8:]
    print(ys, body[::step])
