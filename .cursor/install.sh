#!/usr/bin/env bash
#
# Cloud Agent environment bootstrap for the Kamaz-Leveler ESP32 firmware.
#
# Installs the Arduino CLI toolchain, the Espressif ESP32 core, and every
# library the sketch (kamaz_8_5_7_fr_adafruiti_OTA_NM.ino) includes so that
# the firmware can be compiled with:
#
#   arduino-cli compile -b esp32:esp32:esp32 <sketch-folder>
#
# The script is idempotent: rerunning it converges to the same toolchain
# without duplicating configuration or rebuilding what is already present.

set -euo pipefail

ARDUINO_CLI_VERSION="1.5.1"
ESP32_CORE_VERSION="3.3.12"
ESP32_INDEX_URL="https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"

# Arduino libraries the sketch depends on, pinned for reproducibility.
# U8g2 is not used by the firmware directly, but the GEM menu library always
# compiles its U8g2 backend (GEM_u8g2.cpp), so the header must be resolvable.
LIBRARIES=(
  "ArduinoJson@7.4.3"
  "Adafruit BusIO@1.17.4"
  "Adafruit GFX Library@1.12.6"
  "Adafruit ST7735 and ST7789 Library@1.11.0"
  "Adafruit ADS1X15@2.6.2"
  "EncButton@3.7.6"
  "GEM@1.8.1"
  "GyverFilters@3.2.0"
  "MPU6050@1.4.5"
  "U8g2@2.36.19"
)

log() { printf '\n=== %s ===\n' "$1"; }

install_arduino_cli() {
  if command -v arduino-cli >/dev/null 2>&1; then
    log "arduino-cli already installed: $(arduino-cli version)"
    return
  fi

  log "Installing arduino-cli ${ARDUINO_CLI_VERSION}"
  # Prefer a location already on PATH; fall back to a user-local bin.
  local bindir="/usr/local/bin"
  local sudo=""
  if [ -w "$bindir" ]; then
    sudo=""
  elif sudo -n true 2>/dev/null; then
    sudo="sudo"
  else
    bindir="${HOME}/.local/bin"
    mkdir -p "$bindir"
    case ":${PATH}:" in
      *":${bindir}:"*) : ;;
      *) export PATH="${bindir}:${PATH}" ;;
    esac
  fi

  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
    | ${sudo} env BINDIR="$bindir" sh -s "$ARDUINO_CLI_VERSION"

  arduino-cli version
}

configure_board_index() {
  log "Configuring ESP32 board manager index"
  if [ ! -f "${HOME}/.arduino15/arduino-cli.yaml" ]; then
    arduino-cli config init
  fi
  # `config set` replaces the list, so this stays idempotent across reruns.
  arduino-cli config set board_manager.additional_urls "$ESP32_INDEX_URL"
  arduino-cli config set library.enable_unsafe_install true
}

install_core() {
  log "Installing ESP32 core ${ESP32_CORE_VERSION}"
  arduino-cli core update-index
  arduino-cli core install "esp32:esp32@${ESP32_CORE_VERSION}"
}

install_libraries() {
  log "Installing Arduino libraries"
  arduino-cli lib update-index
  arduino-cli lib install "${LIBRARIES[@]}"
}

main() {
  install_arduino_cli
  configure_board_index
  install_core
  install_libraries

  log "Toolchain summary"
  arduino-cli version
  arduino-cli core list
  echo "Environment ready. Compile the firmware with:"
  echo "  arduino-cli compile -b esp32:esp32:esp32 <sketch-folder>"
}

main "$@"
