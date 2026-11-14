import sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
print("start", flush=True)
import pcbnew
print("imported", pcbnew.Version(), flush=True)
print("F_Cu const", pcbnew.F_Cu, "B_Cu", pcbnew.B_Cu, "F_Cu id", pcbnew.F_Cu, flush=True)
