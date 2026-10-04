from pathlib import Path

p = Path(r"c:\my\for cursor\kamaz_leveler\kamaz_9_3_0_fr_adafruiti_OTA_NM\kamaz_9_3_0_fr_adafruiti_OTA_NM.ino")
lines = p.read_text(encoding="utf-8").splitlines(keepends=True)
start = next(i for i, l in enumerate(lines) if l.startswith("bool downloadGitHubFirmware"))
end = next(i for i, l in enumerate(lines) if i > start and l.startswith("bool checkGitHubUpdate"))
n = 0
for i in range(start, end):
    if "otaInProgress = false;" in lines[i]:
        indent = lines[i][: len(lines[i]) - len(lines[i].lstrip())]
        lines[i] = indent + 'githubOtaEndInstallSession("dl-exit");\n'
        n += 1
for i in range(start, end):
    if "OK set_boot" in lines[i]:
        for j in range(i, min(i + 15, end)):
            if lines[j].strip() == "return true;":
                indent = lines[j][: len(lines[j]) - len(lines[j].lstrip())]
                lines.insert(j, indent + 'githubOtaEndInstallSession("dl-ok");\n')
                n += 1
                break
        break
p.write_text("".join(lines), encoding="utf-8")
print(f"replaced {n} in lines {start+1}-{end+1}")
