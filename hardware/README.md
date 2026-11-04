# Kamaz Leveler — аппаратная часть (PCB)

Плата контроллера пневмоподвески Kamaz Leveler: **ESP32-C3**, 2 слоя, детали с двух сторон.
Проект KiCad 10.0.7. Механика согласована с корпусом **15-8** (чертёж `k_15-08-...pdf`).

## Текущий статус

| Параметр | Значение |
|---|---|
| Схема | `kicad/kamaz_leveler_hw.kicad_sch` — 113 компонентов (U7 = ESP32-C3-DevKitC-02) |
| Плата | `kicad/kamaz_leveler_hw.kicad_pcb` — 83 × 58 мм, 2 слоя, 117 футпринтов |
| Корпус | 4×Ø2,2 NPTH по контуру **28 × 61,6**; модуль сдвинут на −5 мм; J1/J2 — низ (окно 62×28), J3…J8 — верх (окно 42×20) |
| Разводка | **275 из 284 цепей (96,8%)**, 2394 трассы/вии |
| Электрика DRC | **0 замыканий, 0 нарушений зазора** (правило 0,127 мм) |
| Не разведено | 9 цепей: `/BUT1`, `/VIN_FUSED`, `/VLV_IN2,3,5`, `/ST_U3_34`, `/PRES_IN`, `/VLV_DRV1`, `/VLV_DRV4` |
| Прочие DRC | `copper_edge_clearance` 19, `track_width` 4, `via_dangling` 15, silk ~215, courtyard под модулем (намеренно) |
| Производство | `production/` — Gerber, Excellon, pos.csv, STEP, PDF, BOM (перегенерируется скриптом) |

> **Статус: не финальный релиз.** До производства осталось закрыть 9 цепей и почистить `copper_edge`/`via_dangling`.

## Структура

```
hardware/
├── README.md            ← этот файл
├── HANDOFF.md           ← журнал работ (история решений и бэкапов)
├── kicad/               ← сам проект KiCad
│   ├── kamaz_leveler_hw.kicad_pro / .sch / .pcb / .prl
│   ├── kamaz_leveler_hw.kicad_dru   ← кастомные правила DRC
│   ├── BOM.csv
│   ├── fp-lib-table, sym-lib-table  ← локальные библиотеки (${KIPRJMOD})
│   ├── kamaz.pretty/, kamaz_leveler_hw.pretty/
│   ├── kamaz_leveler.kicad_sym
│   └── 3dmodels/        ← STEP-модели компонентов
├── docs/                ← HARDWARE.md, CALCULATIONS.md, ТЗ, таблица расстановки
├── images/              ← актуальные рендеры платы и схемы
├── production/          ← выходные файлы для завода (Gerber/сверловка/pos/STEP/PDF/BOM)
├── scripts/             ← экспорт и рендер (см. ниже)
├── 3d/                  ← сборка платы в STEP
└── backups/             ← ключевые бэкапы платы (даты в имени)
```

## Как открыть

1. KiCad 10.0.7 → **File → Open Project** → `kicad/kamaz_leveler_hw.kicad_pro`.
2. Библиотеки подключаются автоматически: `${KIPRJMOD}` указывает на `kicad/`, таблицы `fp-lib-table`/`sym-lib-table` уже в проекте.
3. DRC: `Inspect → Design Rules Checker` (правила — `kamaz_leveler_hw.kicad_dru`).

## Как перегенерировать производство

```bash
"C:\Program Files\KiCad\10.0\bin\python.exe" scripts\export_prod.py
```

Скрипт кладёт в `production/`: Gerber (2 слоя, без Protel-расширений), Excellon + карты сверловки, `pos.csv`, `board.step`, `board.pdf`, `board_stats.txt`, `schematic.pdf`, `BOM.csv`.

## Зависимости

| Что | Где |
|---|---|
| KiCad 10.0.7 (`kicad-cli`, `pcbnew` Python) | `C:\Program Files\KiCad\10.0\` |
| Python с `pcbnew` | `C:\Program Files\KiCad\10.0\bin\python.exe` |
| FreeRouting 2.5.0 + JRE 25 | `C:\my\freerouting\` |
| PIL (для рендеров) | системный Python |

## Известные ограничения / waivers

- **Courtyard под модулем ESP32** — детали на обратной стороне под модулем (высота ≤5 мм) стоят намеренно; DRC это помечает.
- **Silk** — пересечения шелкографии с падами/друг другом: косметика, на электрику не влияет.
- **9 несоединённых цепей** — список выше; для 2 слоёв это остаток после автотрассировки, требует ручной доработки.

## История

- 2026-11-01: перепроектирование под корпус 15-8, новая расстановка (SA по HPWL с весами «трудных» цепей), 275/284.
- 2026-11-04: проект перенесён в этот репозиторий (`hardware/`), структура упорядочена, производство перегенерировано.

Подробный журнал — `HANDOFF.md`.
