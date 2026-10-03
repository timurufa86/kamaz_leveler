# -*- coding: utf-8 -*-
"""Generate menu_tree_10_6_2.xlsx for user edits (stable ids for later apply)."""
from pathlib import Path

from openpyxl import Workbook
from openpyxl.styles import Alignment, Font, PatternFill, Border, Side
from openpyxl.utils import get_column_letter
from openpyxl.worksheet.datavalidation import DataValidation

OUT = Path(__file__).resolve().parents[1] / "menu_tree_10_6_4.xlsx"

# Columns for menu sheet
MENU_COLS = [
    "id",
    "path",
    "order",
    "type",
    "label",
    "unit",
    "min",
    "max",
    "step",
    "options",
    "notes",
    "new_label",
    "new_unit",
    "new_min",
    "new_max",
    "new_step",
    "new_options",
    "new_path",
    "new_order",
    "action",
    "comment",
]

# (id, path, order, type, label, unit, min, max, step, options, notes)
ROWS = [
    # Main
    ("linkSystem", "Главное", 1, "link", "Система", "—", "", "", "", "", "→ страница Система"),
    ("linkValveBlock", "Главное", 2, "link", "Клапанный блок", "—", "", "", "", "", "→ страница Клапанный блок"),
    ("linkAuto", "Главное", 3, "link", "Авторежим", "—", "", "", "", "", "→ страница Авторежим"),
    ("linkMovement", "Главное", 4, "link", "Движение", "—", "", "", "", "", "→ страница Движение"),
    ("linkImu", "Главное", 5, "link", "MPU", "—", "", "", "", "", "→ страница MPU / фильтры"),
    ("linkSettings", "Главное", 6, "link", "Просмотр", "—", "", "", "", "", "→ страница Просмотр"),
    # System
    ("itemReset", "Главное / Система", 1, "button", "Сброс ошибок", "—", "", "", "", "", ""),
    ("itemWiFi", "Главное / Система", 2, "button", "Настроить WiFi", "—", "", "", "", "", ""),
    ("linkOta", "Главное / Система", 3, "link", "Обновления", "—", "", "", "", "", "→ Обновления"),
    ("linkDisplay", "Главное / Система", 4, "link", "Дисплей", "—", "", "", "", "", "→ Дисплей"),
    ("linkInfo", "Главное / Система", 5, "link", "Информация", "—", "", "", "", "", "→ Информация"),
    # OTA
    ("itemOtaCurrent", "Главное / Система / Обновления", 1, "label", "Версия: …", "—", "", "", "", "", "живая метка; префикс Верс:"),
    ("itemOtaStatus", "Главное / Система / Обновления", 2, "label", "Статус: …", "—", "", "", "", "", "живая метка"),
    ("itemOtaList", "Главное / Система / Обновления", 3, "button", "Список прошивок", "—", "", "", "", "", "→ Прошивки (GitHub)"),
    ("itemOtaInstallLast", "Главное / Система / Обновления", 4, "button", "Установить последнюю", "—", "", "", "", "", ""),
    ("itemOtaArduino", "Главное / Система / Обновления", 5, "button", "Режим ArduinoOTA (Wi-Fi)", "—", "", "", "", "", ""),
    ("itemOtaExit", "Главное / Система / Обновления", 6, "button", "Выход из режима OTA", "—", "", "", "", "", "скрыт если OTA не активен"),
    # OTA list
    ("itemOtaListHdr", "Главное / Система / Обновления / Прошивки", 1, "label", "Статус: …", "—", "", "", "", "", "заголовок списка"),
    ("itemOtaRel0", "Главное / Система / Обновления / Прошивки", 2, "button", "—", "—", "", "", "", "", "слот релиза 0 (тег подставляется)"),
    ("itemOtaRel1", "Главное / Система / Обновления / Прошивки", 3, "button", "—", "—", "", "", "", "", "слот релиза 1"),
    ("itemOtaRel2", "Главное / Система / Обновления / Прошивки", 4, "button", "—", "—", "", "", "", "", "слот релиза 2"),
    # OTA card
    ("itemCardTag", "Главное / Система / Обновления / Прошивки / Релиз", 1, "label", "Релиз: —", "—", "", "", "", "", ""),
    ("itemCardInfo", "Главное / Система / Обновления / Прошивки / Релиз", 2, "label", "данных нет", "—", "", "", "", "", "на экране Инфо:"),
    ("itemCardStatus", "Главное / Система / Обновления / Прошивки / Релиз", 3, "label", "обновите список", "—", "", "", "", "", "на экране Статус:"),
    ("itemCardSha", "Главное / Система / Обновления / Прошивки / Релиз", 4, "label", "SHA-256: не проверен", "—", "", "", "", "", ""),
    ("itemCardInstall", "Главное / Система / Обновления / Прошивки / Релиз", 5, "button", "УСТАНОВИТЬ", "—", "", "", "", "", ""),
    ("itemCardCheckSha", "Главное / Система / Обновления / Прошивки / Релиз", 6, "button", "Проверить SHA-256", "—", "", "", "", "", ""),
    # Display
    ("itemContrast", "Главное / Система / Дисплей", 1, "spinner", "Яркость,%", "%", 10, 100, 1, "", "живой preview PWM"),
    ("itemBacklightOff", "Главное / Система / Дисплей", 2, "spinner", "Приглуш.,мин", "мин", 0, 30, 1, "", "0 = выкл"),
    ("itemFrameMs", "Главное / Система / Дисплей", 3, "spinner", "Интервал,мс", "мс", 20, 200, 5, "", "интервал кадра"),
    ("itemRedrawAngle", "Главное / Система / Дисплей", 4, "spinner", "Порог углов,гра", "°", 0.01, 0.5, 0.01, "", "порог перерисовки"),
    ("itemRedrawPressure", "Главное / Система / Дисплей", 5, "spinner", "Порог давл,бар", "бар", 0.01, 0.5, 0.01, "", "порог перерисовки"),
    # Info
    ("itemInfoVersion", "Главное / Система / Информация", 1, "label", "Версия: …", "—", "", "", "", "", "VERSION"),
    ("itemInfoMode", "Главное / Система / Информация", 2, "label", "Режим: …", "—", "", "", "", "", "АВТО/ДВИЖ/РУЧ"),
    ("itemInfoMpu", "Главное / Система / Информация", 3, "label", "MPU: …", "—", "", "", "", "", "OK/ОШИБКА"),
    ("itemInfoMaster", "Главное / Система / Информация", 4, "label", "МП: …", "бар", "", "", "", "", "мастер-давление"),
    ("itemInfoWiFi", "Главное / Система / Информация", 5, "label", "Wi-Fi: …", "—", "", "", "", "", ""),
    ("itemInfoSystem", "Главное / Система / Информация", 6, "label", "Аптайм: …", "—", "", "", "", "", "часы + RAM"),
    ("itemInfoErrors", "Главное / Система / Информация", 7, "label", "Ошибки: …", "—", "", "", "", "", ""),
    # Valve block
    ("linkPressure", "Главное / Клапанный блок", 1, "link", "Давление", "—", "", "", "", "", "→ Давление"),
    ("linkValve", "Главное / Клапанный блок", 2, "link", "Клапаны", "—", "", "", "", "", "→ Клапаны"),
    ("linkTest", "Главное / Клапанный блок", 3, "link", "Тестирование", "—", "", "", "", "", "→ Тестирование"),
    # Pressure
    ("itemPressureMin", "Главное / Клапанный блок / Давление", 1, "spinner", "min Р под, бар", "бар", 0.1, 5.0, 0.1, "", ""),
    ("itemPressureMax", "Главное / Клапанный блок / Давление", 2, "spinner", "max Р под., бар", "бар", 1.0, 8.0, 0.1, "", ""),
    ("itemMasterLow", "Главное / Клапанный блок / Давление", 3, "spinner", "min P Маг., бар", "бар", 0.0, 4.0, 0.1, "", "подпись 0.00…4.00; пишет в рантайм сразу"),
    ("itemDeadband", "Главное / Клапанный блок / Давление", 4, "spinner", "Гистерезис, бар", "бар", 0.05, 0.5, 0.05, "", ""),
    ("itemMasterCheck", "Главное / Клапанный блок / Давление", 5, "spinner", "Проверка P Маг., мин.", "мин", 1, 10, 1, "", "в конфиге секунды = мин×60"),
    ("itemPressStabilize", "Главное / Клапанный блок / Давление", 6, "spinner", "Выравн.МП,мс", "мс", 100, 2000, 50, "", ""),
    ("itemPressIdle", "Главное / Клапанный блок / Давление", 7, "spinner", "Цикл опроса, мин", "мин", 2, 30, 1, "", ""),
    # Valves
    ("itemReleaseDelay", "Главное / Клапанный блок / Клапаны", 1, "spinner", "Сброс,с", "с", 1, 10, 1, "", "импульс сброса"),
    ("itemInflateDelay", "Главное / Клапанный блок / Клапаны", 2, "spinner", "Накачка,с", "с", 1, 10, 1, "", "импульс накачки"),
    ("itemManualMaxTime", "Главное / Клапанный блок / Клапаны", 3, "spinner", "Макс.время,с", "с", 1, 30, 1, "", "лимит ручной операции"),
    # Test
    ("itemTest", "Главное / Клапанный блок / Тестирование", 1, "button", "Запустить тест", "—", "", "", "", "", "только MANUAL, без ошибок"),
    ("itemValveTestManual", "Главное / Клапанный блок / Тестирование", 2, "button", "Тест клапанов (ручной)", "—", "", "", "", "", ""),
    # Auto
    ("itemTiltX", "Главное / Авторежим", 1, "spinner", "Поперечный,гра", "°", 0.0, 3.0, 0.1, "", ""),
    ("itemTiltY", "Главное / Авторежим", 2, "spinner", "Продольный,гра", "°", 0.0, 3.0, 0.1, "", ""),
    ("itemCoarseZone", "Главное / Авторежим", 3, "spinner", "Грубая,пор.", "доля", 0.2, 0.9, 0.05, "", "множитель (setMultiplySep)"),
    ("itemFineZone", "Главное / Авторежим", 4, "spinner", "Точная,пор.", "доля", 0.05, 0.3, 0.01, "", "множитель"),
    ("itemWorsening", "Главное / Авторежим", 5, "spinner", "Хуже", "—", 1.05, 2.0, 0.05, "", "множитель; ед. в подписи нет"),
    ("itemNivCount", "Главное / Авторежим", 6, "spinner", "Попыток в час", "шт", 1, 20, 1, "", ""),
    ("itemTimeInterval", "Главное / Авторежим", 7, "spinner", "Интервал,мин", "мин", 1, 60, 1, "", ""),
    # Movement
    ("itemMovementEnabled", "Главное / Движение", 1, "checkbox", "Режим Движение", "вкл/выкл", "", "", "", "", "по умолчанию вкл"),
    ("itemMovementFront", "Главное / Движение", 2, "spinner", "Давл.перед,бар", "бар", 1.0, 6.0, 0.1, "", ""),
    ("itemMovementRear", "Главное / Движение", 3, "spinner", "Давл.зад,бар", "бар", 1.0, 6.0, 0.1, "", ""),
    ("itemParkingPressure", "Главное / Движение", 4, "spinner", "Давл.стоянки,бар", "бар", 0.0, 6.0, 0.1, "", "0 = как в движении"),
    ("itemMoveDuration", "Главное / Движение", 5, "spinner", "До входа,с", "с", 10, 120, 5, "", ""),
    ("itemMoveSettle", "Главное / Движение", 6, "spinner", "Успокоение,с", "с", 10, 120, 5, "", ""),
    ("itemMoveCheck", "Главное / Движение", 7, "spinner", "Проверка давл.,с", "с", 30, 300, 10, "", ""),
    ("itemMoveTolerance", "Главное / Движение", 8, "spinner", "Допуск давл.,бар", "бар", 0.1, 1.0, 0.05, "", ""),
    # MPU
    ("itemImuZero", "Главное / MPU", 1, "button", "Обнулить углы", "—", "", "", "", "", ""),
    ("itemImuCalib", "Главное / MPU", 2, "button", "Калибровка офсетов", "—", "", "", "", "", ""),
    ("itemImuDiag", "Главное / MPU", 3, "button", "Диагностика MPU", "—", "", "", "", "", ""),
    ("itemImuAccelFs", "Главное / MPU", 4, "spinner", "Диапазон accel", "G", 0, 3, 1, "±2G|±4G|±8G|±16G", "индекс 0…3"),
    ("itemImuDlpf", "Главное / MPU", 5, "spinner", "Фильтр DLPF", "Гц", 0, 6, 1, "256 Гц|188 Гц|98 Гц|42 Гц|20 Гц|10 Гц|5 Гц", "индекс 0…6"),
    ("itemImuMotionDet", "Главное / MPU", 6, "spinner", "Чувств. MOT", "мг", 20, 255, 5, "", "подпись thr×2 мг"),
    ("itemGyroThr", "Главное / MPU", 7, "spinner", "Чувств. вибрации", "1…25", 1, 25, 1, "", "слаб≤8 / норм≤16 / жёстк"),
    ("itemGyroBump", "Главное / MPU", 8, "spinner", "Чувств. неровн.", "1…25", 1, 25, 1, "", "слаб/норм/жёстк"),
    ("itemAccelThr", "Главное / MPU", 9, "spinner", "Чувств. ускорения", "LSB", 0, 4, 1, "200 слаб|500 норм|1000|1500|2000 жёстк", "индекс 0…4"),
    ("itemZeroAngleX", "Главное / MPU", 10, "spinner", "Нуль крен,гра", "°", -45.0, 45.0, 0.01, "", ""),
    ("itemZeroAngleY", "Главное / MPU", 11, "spinner", "Нуль тангаж,гра", "°", -45.0, 45.0, 0.01, "", ""),
    ("itemImuPreset", "Главное / MPU", 12, "spinner", "Плавность углов", "—", 0, 2, 1, "Плавно|Быстро|Баланс", "грузит скрытые Kalman/EMA/slew"),
    ("itemImuPollMs", "Главное / MPU", 13, "spinner", "Опрос,мс", "мс", 15, 100, 5, "", ""),
    ("itemImuSlew", "Главное / MPU", 14, "spinner", "Макс.скорость,гра/с", "°/с", 5, 120, 5, "", ""),
    # View
    ("itemSet1", "Главное / Просмотр", 1, "label", "Давление: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet2", "Главное / Просмотр", 2, "label", "Наклон: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet3", "Главное / Просмотр", 3, "label", "Клапаны: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet4", "Главное / Просмотр", 4, "label", "Авторежим: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet5", "Главное / Просмотр", 5, "label", "Движение: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet6", "Главное / Просмотр", 6, "label", "Дисплей: …", "—", "", "", "", "", "живая сводка"),
    ("itemSet7", "Главное / Просмотр", 7, "label", "MPU: …", "—", "", "", "", "", "живая сводка"),
]

PAGES = [
    ("mainPage", "Главное меню", "Главное"),
    ("systemPage", "Система", "Главное / Система"),
    ("valveBlockPage", "Клапанный блок", "Главное / Клапанный блок"),
    ("autoPage", "Авторежим", "Главное / Авторежим"),
    ("movementPage", "Движение", "Главное / Движение"),
    ("imuPage", "MPU / фильтры", "Главное / MPU"),
    ("settingsViewPage", "Просмотр", "Главное / Просмотр"),
    ("otaPage", "Обновления", "Главное / Система / Обновления"),
    ("otaListPage", "Прошивки (GitHub)", "Главное / Система / Обновления / Прошивки"),
    ("otaCardPage", "Релиз", "Главное / Система / Обновления / Прошивки / Релиз"),
    ("displayPage", "Дисплей", "Главное / Система / Дисплей"),
    ("infoPage", "Информация", "Главное / Система / Информация"),
    ("pressurePage", "Давление", "Главное / Клапанный блок / Давление"),
    ("valvePage", "Клапаны", "Главное / Клапанный блок / Клапаны"),
    ("testPage", "Тестирование", "Главное / Клапанный блок / Тестирование"),
]


def style_header(ws, ncols):
    fill = PatternFill("solid", fgColor="1F4E79")
    font = Font(color="FFFFFF", bold=True)
    thin = Border(
        left=Side(style="thin", color="B0B0B0"),
        right=Side(style="thin", color="B0B0B0"),
        top=Side(style="thin", color="B0B0B0"),
        bottom=Side(style="thin", color="B0B0B0"),
    )
    for c in range(1, ncols + 1):
        cell = ws.cell(1, c)
        cell.fill = fill
        cell.font = font
        cell.alignment = Alignment(wrap_text=True, vertical="center")
        cell.border = thin
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions


def autosize(ws, max_width=56):
    for col in ws.columns:
        letter = get_column_letter(col[0].column)
        length = 0
        for cell in col:
            if cell.value is None:
                continue
            length = max(length, len(str(cell.value)))
        ws.column_dimensions[letter].width = min(max(length + 2, 10), max_width)


def main():
    wb = Workbook()

    # --- README ---
    ws = wb.active
    ws.title = "README"
    lines = [
        "Kamaz-Leveler menu tree — V 10.6.4",
        "",
        "КАК ПРАВИТЬ",
        "1. Открой лист menu — там все пункты в порядке экрана.",
        "2. Колонку id НЕ меняй и НЕ удаляй — по ней код найдёт пункт.",
        "3. Править можно только колонки new_* , action и comment.",
        "4. Текущие label/unit/min/max/step/options — справочные (как сейчас в прошивке).",
        "",
        "КОЛОНКА action (оставь пустой = без изменений по этой строке):",
        "  rename  — только название/ед. (new_label, new_unit, new_options)",
        "  limits  — только пределы (new_min, new_max, new_step)",
        "  both    — и название, и пределы",
        "  move    — перенос: заполни new_path и/или new_order",
        "  delete  — убрать пункт из меню",
        "  add     — новая строка: заполни id (уникальный), path, order, type, new_*",
        "",
        "Если заполнил любой new_* без action — будет применено как both/rename по заполненным полям.",
        "",
        "ЛИСТ pages — переименование заголовков страниц (new_title).",
        "",
        "После правок пришли этот же xlsx в чат. Агент применит изменения в ui_menu_build.cpp.",
        "",
        "Версия прошивки-источника: V 10.6.4 FreeRTOS OTA",
        "Файл сгенерирован tools/gen_menu_tree_xlsx.py",
    ]
    for i, line in enumerate(lines, 1):
        ws.cell(i, 1, line)
        if i == 1:
            ws.cell(i, 1).font = Font(bold=True, size=14)
    ws.column_dimensions["A"].width = 110

    # --- menu ---
    wm = wb.create_sheet("menu")
    for c, name in enumerate(MENU_COLS, 1):
        wm.cell(1, c, name)
    edit_fill = PatternFill("solid", fgColor="FFF2CC")  # yellow = editable
    readonly_fill = PatternFill("solid", fgColor="E7E6E6")
    edit_cols = {
        "new_label",
        "new_unit",
        "new_min",
        "new_max",
        "new_step",
        "new_options",
        "new_path",
        "new_order",
        "action",
        "comment",
    }
    for r, row in enumerate(ROWS, 2):
        data = dict(zip(
            ["id", "path", "order", "type", "label", "unit", "min", "max", "step", "options", "notes"],
            row,
        ))
        for c, name in enumerate(MENU_COLS, 1):
            val = data.get(name, "")
            cell = wm.cell(r, c, val if val != "" else None)
            if name in edit_cols:
                cell.fill = edit_fill
            elif name == "id":
                cell.fill = PatternFill("solid", fgColor="FCE4D6")
                cell.font = Font(bold=True)
            else:
                cell.fill = readonly_fill
    style_header(wm, len(MENU_COLS))
    # highlight header of editable cols
    for c, name in enumerate(MENU_COLS, 1):
        if name in edit_cols:
            wm.cell(1, c).fill = PatternFill("solid", fgColor="BF8F00")
    autosize(wm)
    wm.column_dimensions["A"].width = 22
    wm.column_dimensions["B"].width = 48
    # label + new_label: широкие — длинные русские названия не обрезаются в Excel
    wm.column_dimensions["E"].width = 36
    wm.column_dimensions["L"].width = 36
    wm.column_dimensions["J"].width = 40
    wm.column_dimensions["K"].width = 36
    for r in range(2, len(ROWS) + 2):
        wm.row_dimensions[r].height = 18
        for col in ("E", "L", "J", "K", "U"):
            wm[f"{col}{r}"].alignment = Alignment(wrap_text=True, vertical="center")

    dv = DataValidation(
        type="list",
        formula1='"rename,limits,both,move,delete,add"',
        allow_blank=True,
    )
    dv.error = "Выбери значение из списка"
    dv.errorTitle = "action"
    wm.add_data_validation(dv)
    dv.add(f"T2:T{len(ROWS) + 50}")

    # --- pages ---
    wp = wb.create_sheet("pages")
    page_cols = ["page_id", "title", "path", "new_title", "action", "comment"]
    for c, name in enumerate(page_cols, 1):
        wp.cell(1, c, name)
    for r, (page_id, title, path) in enumerate(PAGES, 2):
        wp.cell(r, 1, page_id).fill = PatternFill("solid", fgColor="FCE4D6")
        wp.cell(r, 1).font = Font(bold=True)
        wp.cell(r, 2, title).fill = readonly_fill
        wp.cell(r, 3, path).fill = readonly_fill
        for c in (4, 5, 6):
            wp.cell(r, c).fill = edit_fill
    style_header(wp, len(page_cols))
    for c in (4, 5, 6):
        wp.cell(1, c).fill = PatternFill("solid", fgColor="BF8F00")
    autosize(wp)
    dv2 = DataValidation(type="list", formula1='"rename,"', allow_blank=True)
    wp.add_data_validation(dv2)
    dv2.add(f"E2:E{len(PAGES) + 20}")

    # --- legend ---
    wl = wb.create_sheet("legend")
    legend = [
        ("Цвет", "Значение"),
        ("Оранжевый id", "НЕ МЕНЯТЬ — ключ для применения в коде"),
        ("Серый", "Текущее состояние прошивки (справочно)"),
        ("Жёлтый", "Поля для твоих правок"),
        ("", ""),
        ("type", "смысл"),
        ("link", "переход на другую страницу"),
        ("spinner", "числовое значение с min/max/step"),
        ("checkbox", "вкл/выкл"),
        ("button", "действие по нажатию"),
        ("label", "только чтение / живая строка"),
    ]
    for r, (a, b) in enumerate(legend, 1):
        wl.cell(r, 1, a)
        wl.cell(r, 2, b)
        if r == 1:
            wl.cell(r, 1).font = Font(bold=True)
            wl.cell(r, 2).font = Font(bold=True)
    wl.column_dimensions["A"].width = 18
    wl.column_dimensions["B"].width = 55

    wb.save(OUT)
    print(f"Wrote {OUT} ({len(ROWS)} menu rows, {len(PAGES)} pages)")


if __name__ == "__main__":
    main()
