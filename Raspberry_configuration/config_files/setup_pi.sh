#!/bin/bash

# Sprawdzenie, czy skrypt został uruchomiony z uprawnieniami root
if [ "$EUID" -ne 0 ]; then
  echo "Proszę uruchomić skrypt jako root (np. sudo ./setup_pi.sh)"
  exit 1
fi

# Pobranie bezwzględnej ścieżki katalogu repozytorium
SCRIPT_DIR=$(dirname "$(realpath "$0")")


echo "--- 1. Bezpieczna konfiguracja parametrów startowych kernela ---"
CMDLINE_FILE="/boot/firmware/cmdline.txt"

if [ ! -f "$CMDLINE_FILE" ]; then
    echo "Błąd: Nie znaleziono pliku $CMDLINE_FILE!"
else
    cp "$CMDLINE_FILE" "${CMDLINE_FILE}.bak"
    echo "Utworzono kopię zapasową w ${CMDLINE_FILE}.bak"

    # Lista dodatkowych parametrów z Twojego starego pliku
    PARAMS_TO_ADD=(
        "quiet"
        "splash"
        "plymouth.ignore-serial-consoles"
        "cfg80211.ieee80211_regdom=PL"
    )

    for param in "${PARAMS_TO_ADD[@]}"; do
        # Sprawdzamy, czy parametr znajduje się już w pliku (-q wycisza wyjście, -w szuka całego słowa)
        if ! grep -q -w "$param" "$CMDLINE_FILE"; then
            sed -i "1 s/$/ $param/" "$CMDLINE_FILE"
            echo "Dodano parametr: $param"
        else
            echo "Parametr $param już istnieje, pomijam."
        fi
    done
    
    echo "Konfiguracja cmdline.txt zakończona sukcesem."
fi

echo "--- 2. Konfiguracja sprzętowa ---"
if [ -f "$SCRIPT_DIR/config.txt" ]; then
    cp "$SCRIPT_DIR/config.txt" /boot/firmware/config.txt
    echo "Plik config.txt nadpisany."
fi

echo "--- 3. Instalacja wymaganych pakietów ---"
if [ -f "$SCRIPT_DIR/manual_packages.txt" ]; then
    apt-get update
    PACKAGES=$(tr '\n' ' ' < "$SCRIPT_DIR/apt_packages.txt")
    # Dodano pakiety python3-venv i python3-pip niezbędne do utworzenia środowiska wirtualnego
    apt-get install -y $PACKAGES
    echo "Pakiety zainstalowane."
    sudo apt-get remove modemmanager -y
    echo "Odinstalowano modemanager"
else
    echo "Błąd: Brak pliku apt_packages.txt"
fi

echo "--- 4. Konfiguracja grup użytkownika ---"
if [ -f "$SCRIPT_DIR/user_groups.txt" ]; then
    USER_LINE=$(cat "$SCRIPT_DIR/user_groups.txt")
    USERNAME=$(echo "$USER_LINE" | awk -F': ' '{print $1}' | tr -d ' ')
    GROUPS=$(echo "$USER_LINE" | awk -F': ' '{print $2}' )
    echo "Grupy uzytkownika: $GROUPS"
    echo "Obecny użytkownik: $USERNAME"
    if ! id -u "$USERNAME" > /dev/null 2>&1; then
        useradd -m -s /bin/bash "$USERNAME"
        echo "$USERNAME:raspi" | chpasswd
        echo "Utworzono nowego użytkownika: $USERNAME"
    fi
    usermod -aG "$GROUPS" "$USERNAME"
    echo "Grupy zostały zaktualizowane."
else
    echo "Błąd: Brak pliku user_groups.txt"
fi

echo "--- 5. Konfiguracja reguł udev (porty UART) ---"
UDEV_SOURCE="$SCRIPT_DIR/99-tty-raw.rules"
UDEV_DEST="/etc/udev/rules.d/99-tty-raw.rules"

if [ -f "$UDEV_SOURCE" ]; then
    cp "$UDEV_SOURCE" "$UDEV_DEST"
    chmod 644 "$UDEV_DEST"
    echo "Skopiowano reguły udev do $UDEV_DEST"

    udevadm control --reload-rules
    udevadm trigger
    echo "Przeładowano i zaaplikowano reguły udev."
else
    echo "Błąd: Nie znaleziono pliku 99-tty-raw.rules w katalogu skryptu!"
fi

echo "--- 6. Konfiguracja bezhasłowego sudo dla użytkownika $USERNAME ---"
SUDOERS_FILE="/etc/sudoers.d/010_$USERNAME-nopasswd"

if [ ! -f "$SUDOERS_FILE" ]; then
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > "$SUDOERS_FILE"
    # Pliki w sudoers.d BEZWZGLĘDNIE muszą mieć uprawnienia 0440, 
    # w przeciwnym razie system zablokuje całe sudo!
    chmod 0440 "$SUDOERS_FILE"
    echo "Dodano bezhasłowe sudo dla użytkownika $USERNAME."
else
    echo "Plik $SUDOERS_FILE już istnieje. Pomijam."
fi

USER_HOME="/home/$USERNAME"
echo "Tworzenie wirtualnego środowiska w $USER_HOME/Mavlink..."
# Wykonanie komend w imieniu użytkownika (np. pi5), a nie jako root!
sudo -u "$USERNAME" bash -c "
    mkdir -p $USER_HOME/Mavlink
    cd $USER_HOME/Mavlink
    python3 -m venv venvMavlink
    chmod u+x ./venvMavlink/bin/activate
    source ./venvMavlink/bin/activate
    echo "aktywowano środowisko wirtualne"
    pip install -r '$SCRIPT_DIR/requirements.txt'

"

echo "--- ZAKOŃCZONO ---"
echo "Zależności Pythona zostały zainstalowane. Aby zastosować wszystkie zmiany, wykonaj:"
echo "sudo reboot"

