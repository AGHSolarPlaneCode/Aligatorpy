#!/usr/bin/env python3
"""
fix_flight_csv.py
==================
Naprawia CSV lotu uszkodzony przez NUL-e (bajty zerowe).

Przyczyna: przy przerwaniu zapisu (Ctrl+C, zanik zasilania) system plików
potrafi zostawić w pliku blok bajtów zerowych zamiast faktycznie zapisanych
danych. Python's csv reader wywala się wtedy z "_csv.Error: line contains NUL".

Skrypt usuwa NUL-e, odrzuca niekompletne/uszkodzone wiersze i zapisuje czysty
plik (oryginał zachowuje jako .bak).

UŻYCIE:
    python3 fix_flight_csv.py ~/Documents/flights/no_gps/2026-08-13_19-51-48/frames.csv

    # albo wskaż folder lotu - sam znajdzie frames.csv / photos_position.csv
    python3 fix_flight_csv.py ~/Documents/flights/no_gps/2026-08-13_19-51-48
"""

import argparse
import csv
import os
import shutil
import sys
from pathlib import Path

CSV_NAMES = ["frames.csv", "photos_position.csv"]


def find_csv(path: Path):
    if path.is_file():
        return path
    for name in CSV_NAMES:
        candidate = path / name
        if candidate.exists():
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", help="Ścieżka do pliku CSV albo do folderu lotu")
    parser.add_argument("--dry-run", action="store_true", help="Pokaż co zostanie zrobione, bez zapisu")
    args = parser.parse_args()

    path = Path(os.path.expanduser(args.path))
    csv_path = find_csv(path)
    if csv_path is None:
        print(f"[error] Nie znalazłem CSV w: {path}")
        sys.exit(1)

    print(f"[info] Naprawiam: {csv_path}")

    with open(csv_path, "rb") as f:
        raw = f.read()

    nul_count = raw.count(b"\x00")
    print(f"[info] Znaleziono {nul_count} bajtów NUL ({nul_count / len(raw) * 100:.1f}% pliku)")

    cleaned = raw.replace(b"\x00", b"")
    text = cleaned.decode("utf-8", errors="replace")

    lines = [ln for ln in text.splitlines() if ln.strip()]
    if not lines:
        print("[error] Po oczyszczeniu plik jest pusty - nie da się uratować")
        sys.exit(1)

    reader = csv.reader(lines)
    rows = list(reader)
    header = rows[0]
    expected_cols = len(header)
    print(f"[info] Nagłówek: {header} ({expected_cols} kolumn)")

    good_rows = []
    bad_rows = 0
    for row in rows[1:]:
        if len(row) == expected_cols and row[0].strip():
            good_rows.append(row)
        else:
            bad_rows += 1

    print(f"[info] Poprawnych wierszy: {len(good_rows)}, odrzuconych: {bad_rows}")

    if args.dry_run:
        print("[dry-run] Nic nie zapisano.")
        return

    backup_path = str(csv_path) + ".bak"
    shutil.copy2(csv_path, backup_path)
    print(f"[info] Kopia oryginału: {backup_path}")

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(good_rows)

    print(f"[done] Zapisano naprawiony plik: {csv_path} ({len(good_rows)} wierszy)")


if __name__ == "__main__":
    main()
