#!/bin/bash

# Sprawdzenie, czy skrypt został uruchomiony z uprawnieniami root
if [ "$EUID" -ne 0 ]; then
  echo "Proszę uruchomić skrypt jako root (np. sudo ./setup_pi.sh)"
  exit 1
fi

# Pobranie bezwzględnej ścieżki katalogu repozytorium
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Dynamiczne wykrywanie użytkownika, który uruchomił skrypt przez sudo
if [ -n "$SUDO_USER" ]; then
    CURRENT_USER="$SUDO_USER"
    USER_HOME="/home/$CURRENT_USER"
else
    # Failback, jeśli skrypt uruchomiono bezpośrednio z konta root
    CURRENT_USER="root"
    USER_HOME="/root"
fi

echo "========================================================"
echo "Konfiguracja systemu dla użytkownika: $CURRENT_USER"
echo "Katalog domowy: $USER_HOME"
echo "========================================================"

echo "--- 1. Bezpieczna konfiguracja parametrów startowych kernela ---"
CMDLINE_FILE="/boot/firmware/cmdline.txt"
if [ -f "$CMDLINE_FILE" ]; then
    cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak"
    echo "Utworzono kopię zapasową w ${CMDLINE_FILE}.bak"

    # Parametry do dodania (wyciszenie bootowania + region WiFi)
    PARAMS_TO_ADD=(
        "quiet"
        "splash"
        "plymouth.ignore-serial-consoles"
        "cfg80211.ieee80211_regdom=PL"
    )

    for param in "${PARAMS_TO_ADD[@]}"; do
        if ! grep -q -w "$param" "$CMDLINE_FILE"; then
            sed -i "1 s/$/ $param/" "$CMDLINE_FILE"
            echo "Dodano parametr: $param"
        else
            echo "Parametr $param już istnieje, pomijam."
        fi
    done
else
    echo "Błąd: Nie znaleziono pliku $CMDLINE_FILE"
fi

usermod -aG dialout,gpio,i2c,spi "$CURRENT_USER"

echo "--- 2. Konfiguracja sprzętowa (config.txt) ---"
if [ -f "$SCRIPT_DIR/config.txt" ]; then
    cp "$SCRIPT_DIR/config.txt" /boot/firmware/config.txt
    echo "Plik config.txt nadpisany."
else
    echo "Ostrzeżenie: Brak pliku config.txt w katalogu skryptu."
fi

echo "--- 3. Instalacja wymaganych pakietów APT ---"
if [ -f "$SCRIPT_DIR/apt_packages.txt" ]; then
    apt-get update
    PACKAGES=$(tr '\n' ' ' < "$SCRIPT_DIR/apt_packages.txt")
    apt-get install -y $PACKAGES
    echo "Pakiety zainstalowane."
    
    # Usunięcie konfliktu z portami szeregowymi
    if dpkg -s modemmanager >/dev/null 2>&1; then
        apt-get remove modemmanager -y
        echo "Odinstalowano modemmanager (częsty konflikt z UART drona)."
    fi
else
    echo "Błąd: Brak pliku apt_packages.txt"
fi

echo "--- 4. Konfiguracja reguł udev dla portów szeregowych UART ---"
UDEV_SOURCE="$SCRIPT_DIR/99-tty-raw.rules"
UDEV_DEST="/etc/udev/rules.d/99-tty-raw.rules"

if [ -f "$UDEV_SOURCE" ]; then
    cp "$UDEV_SOURCE" "$UDEV_DEST"
    chmod 644 "$UDEV_DEST"
    udevadm control --reload-rules
    udevadm trigger
    echo "Reguły udev zostały zaaplikowane."
else
    echo "Ostrzeżenie: Brak pliku 99-tty-raw.rules, pomijam udev."
fi

echo "--- 5. Tworzenie środowiska wirtualnego Pythona (Mavlink) ---"
if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    # Uruchomienie bloku komend jako zalogowany użytkownik (np. pi5), a nie jako root!
    sudo -u "$CURRENT_USER" bash -c "
        mkdir -p '$USER_HOME/Mavlink'
        cd '$USER_HOME/Mavlink'
        python3 -m venv venvMavlink
        source ./venvMavlink/bin/activate
        echo 'Aktywowano środowisko wirtualne venvMavlink.'
        pip install --upgrade pip
        pip install -r '$SCRIPT_DIR/requirements.txt'
    "
    echo "Środowisko wirtualne i zależności zostały pomyślnie skonfigurowane."
else
    echo "Błąd: Brak pliku requirements.txt"
fi

echo "--- ZAKOŃCZONO SUKCESEM ---"
echo "Wszystkie operacje zostały wykonane. Aby zastosować zmiany sprzętowe, zrestartuj Raspberry Pi:"
echo "sudo reboot"