#include "error_handler.h"
#include "config_manager.h"
#include "jhm1200.h"
#include "app_globals.h"

void forceDisplayReset(bool force = false);

SemaphoreHandle_t ErrorHandler::mutex_ = nullptr;
ErrorHandler::ActiveError ErrorHandler::activeErrors[8];
uint8_t ErrorHandler::activeCount = 0;
ErrorHandler::PendingClear ErrorHandler::pendingClear[8];
uint8_t ErrorHandler::pendingCount = 0;
uint8_t ErrorHandler::currentDisplayIndex = 0;
uint32_t ErrorHandler::lastDisplaySwitch = 0;
bool ErrorHandler::hasError = false;

bool cfg_errorIsActive(uint8_t err) { return ErrorHandler::isErrorActive(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorMarkCleared(uint8_t err) { ErrorHandler::markErrorCleared(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorRemove(uint8_t err) { ErrorHandler::removeError(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorHandle(uint8_t err, const char *msg) { ErrorHandler::handleError(static_cast<ErrorHandler::Error>(err), msg); }
void cfg_errorUpdateTime(uint8_t err) { ErrorHandler::updateErrorTime(static_cast<ErrorHandler::Error>(err)); }
bool cfg_errorIsPendingClear(uint8_t err) { return ErrorHandler::isPendingClear(static_cast<ErrorHandler::Error>(err)); }
void cfg_errorCancelClear(uint8_t err) { ErrorHandler::cancelClear(static_cast<ErrorHandler::Error>(err)); }
bool cfg_errorHasActive() { return ErrorHandler::hasActiveErrors(); }
bool cfg_errorHasCriticalPneumatic() { return ErrorHandler::hasCriticalPneumaticErrors(); }

bool cfg_errorHasUiBlocking() { return ErrorHandler::hasUiBlockingErrors(); }
int cfg_errorActiveCount() { return ErrorHandler::getActiveErrorCount(); }
uint8_t cfg_errorCurrent() { return static_cast<uint8_t>(ErrorHandler::getCurrentActiveError()); }
int cfg_errorDisplayIndex() { return ErrorHandler::getCurrentDisplayIndex(); }
const char *cfg_errorMessage(uint8_t err) { return ErrorHandler::getErrorMessage(static_cast<ErrorHandler::Error>(err)); }

void resetSystemErrors() {
    errorScreenBlocking = false;

    int count = ErrorHandler::getActiveErrorCount();

    if (count == 0) {
        forceDisplayReset(true);
        displayDirty = true;
        Serial.println("[SYSTEM] Ошибок нет, обновляем экран");
        return;
    }

    bool anyMarked = false;

    for (int i = 0; i < count; i++) {
        ErrorHandler::Error err = ErrorHandler::getActiveErrorAt(i);
        bool shouldClear = true;

        switch (err) {
            case ErrorHandler::Error::SENSOR:
                {
#if !ENABLE_SIMULATION
                    float bar = 0.0f;
                    bool ok = false;
                    {
                      MutexGuard i2c(xI2CMutex, pdMS_TO_TICKS(120));
                      if (i2c) ok = Jhm1200::readBar(bar);
                    }
                    if (!ok) {
                        shouldClear = false;
                        Serial.println("[SYSTEM] SENSOR сохранена (JHM1200 не отвечает)");
                    } else {
                        Serial.printf("[SYSTEM] SENSOR будет удалена (JHM1200 ok, %.2f бар)\n", bar);
                    }
#else
                    shouldClear = true;
                    Serial.println("[SYSTEM] SENSOR будет удалена (режим симуляции)");
#endif
                    break;
                }

            case ErrorHandler::Error::LOW_PRESSURE:
                {
                    float localMaster;
                    {
                        MutexGuard guard(xStateMutex);
                        if (!guard) {
                            shouldClear = false;
                            break;
                        }
                        localMaster = masterPressure;
                    }
                    float masterLow = ConfigManager::getMasterLowBar();

                    // порог 0 = проверка выкл. — ошибку можно снять
                    if (masterLow > 0.0f && localMaster < masterLow) {
                        shouldClear = false;
                        Serial.printf("[SYSTEM] LOW_PRESSURE сохранена (МП %.2f < %.2f)\n",
                                      localMaster, masterLow);
                    } else {
                        Serial.printf("[SYSTEM] LOW_PRESSURE будет удалена (МП %.2f, порог %.2f)\n",
                                      localMaster, masterLow);
                    }
                    break;
                }

            // Остальные ошибки удаляем всегда
            default:
                Serial.printf("[SYSTEM] Ошибка %d будет удалена\n", (int)err);
                break;
        }

        if (shouldClear) {
            ErrorHandler::markErrorCleared(err);
            anyMarked = true;
            Serial.printf("[SYSTEM] Ошибка %d помечена на удаление\n", (int)err);
        }
    }

    if (!anyMarked) {
        Serial.println("[SYSTEM] Ни одна ошибка не может быть удалена");
    }

    forceDisplayReset(true);
    displayDirty = true;
    Serial.println("[SYSTEM] Сброс ошибок инициирован");
}

