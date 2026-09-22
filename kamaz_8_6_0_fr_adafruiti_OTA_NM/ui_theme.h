#pragma once
#include <stdint.h>

/*
 * UI-тема Kamaz-Leveler: палитра, сетка, метрики (экран 320x240, ST7789).
 *
 * Цвета заданы литералами RGB565, чтобы модуль не зависел от порядка
 * подключения заголовков Adafruit ST77xx.
 */
namespace theme {

/* ---------------------------- палитра ---------------------------- */
constexpr uint16_t BG        = 0x0000;  // фон экрана (чёрный)
constexpr uint16_t PANEL     = 0x1082;  // фон карточек (очень тёмно-серый)
constexpr uint16_t PANEL_ALT = 0x18E3;  // фон шапки/подвала (темно-серый)
constexpr uint16_t BORDER    = 0x4208;  // рамки и разделители
constexpr uint16_t TRACK     = 0x2945;  // дорожка шкалы/прогресса
constexpr uint16_t TEXT      = 0xFFFF;  // основной текст (белый)
constexpr uint16_t TEXT_DIM  = 0x8410;  // второстепенный текст (серый)
constexpr uint16_t ACCENT    = 0x07FF;  // акцент (cyan)
constexpr uint16_t OK        = 0x07E0;  // норма / успех (green)
constexpr uint16_t WARN      = 0xFFE0;  // предупреждение (yellow)
constexpr uint16_t ERR       = 0xF800;  // ошибка / авария (red)
constexpr uint16_t INFO      = 0x001F;  // информация (blue)
constexpr uint16_t SKY       = 0x07FF;  // «небо» авиагоризонта (cyan)
constexpr uint16_t GROUND    = 0xFD20;  // «земля» авиагоризонта (orange)

/* ------------------------- сетка и метрики ------------------------ */
constexpr int16_t SCREEN_W  = 320;
constexpr int16_t SCREEN_H  = 240;

constexpr int16_t MARGIN    = 6;   // внешние отступы
constexpr int16_t GAP       = 4;   // зазор между блоками
constexpr int16_t RADIUS    = 4;   // радиус скругления панелей
constexpr int16_t STATUS_H  = 20;  // высота статус-строки
constexpr int16_t HINT_H    = 18;  // высота строки подсказки
constexpr int16_t ROW_H     = 22;  // стандартная высота строки списка
constexpr int16_t PAD_ROW_H = 30;  // высота карточки подушки на главном экране

}  // namespace theme
