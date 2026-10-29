#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ручная проверка kdi.py: извлечение всех файлов из образов KDI.

Для каждого образа рядом с ним создаётся каталог с именем файла без расширения
(disk/sys.kdi -> disk/sys/) и в него раскладываются все файлы каталога CP/M под
именами ИМЯ.РАСШ. Размер файла — ровно records * 128 байт, т.е. как его видит
сетевая CP/M: последняя запись дополнена тем, что лежит в образе после файла.

Инструмент ручной: ничего не проверяет сам, только выводит список и пишет файлы,
чтобы содержимое можно было посмотреть глазами. Образы открываются на чтение
(как монитором), не изменяются.

    python py/kdi_extract.py                    # все образы в disk/
    python py/kdi_extract.py disk/sys.kdi       # конкретные образы
    python py/kdi_extract.py -o c:\\tmp disk     # раскладывать в другой каталог
"""

import argparse
import glob
import os
import sys

from kdi import KdiDrive

BAD_CHARS = set('\\/:*?"<>|.')  # недопустимо или небезопасно в имени файла


def parse_args():
    p = argparse.ArgumentParser(
        prog="kdi_extract.py",
        description="Извлечение файлов из образов KDI (ручная проверка kdi.py)")
    p.add_argument("images", nargs="*", metavar="KDI",
                   help="образы KDI или каталоги с ними (по умолчанию disk)")
    p.add_argument("-o", metavar="DIR", default="",
                   help="куда создавать подкаталоги (по умолчанию рядом с образом)")
    return p.parse_args()


def find_images(names):
    """Список образов: явные файлы, каталоги и маска *.kdi."""
    found = []
    for name in names or ["disk"]:
        if any(c in name for c in "*?"):
            found.extend(sorted(glob.glob(name)))
        elif os.path.isdir(name):
            found.extend(sorted(glob.glob(os.path.join(name, "*.kdi"))))
        elif os.path.isfile(name):
            found.append(name)
        else:
            print("- %s: не найдено, пропускаем" % name, file=sys.stderr)
    return found


def clean_part(part):
    """Часть имени CP/M (8 или 3 байта) — в безопасное ASCII без мусора."""
    text = part.decode("ascii", "replace").rstrip(" ")
    return "".join(c for c in text if 0x21 <= ord(c) <= 0x7E and c not in BAD_CHARS)


def file_name(raw):
    """11-байтное имя FCB CP/M -> ИМЯ.РАСШ."""
    name = clean_part(raw[:8]) or "NONAME"
    ext = clean_part(raw[8:11])
    return "%s.%s" % (name, ext) if ext else name


def extract(path, out_root):
    """Раскладывает все файлы одного образа; возвращает (файлов, байт)."""
    drv = KdiDrive(path)
    try:
        drv.read_dir()
        stem = os.path.splitext(os.path.basename(path))[0]
        outdir = os.path.join(out_root or os.path.dirname(path), stem)
        os.makedirs(outdir, exist_ok=True)
        print("\n=== %s -> %s" % (path, outdir))
        print("    физ. сектор %d байт, %d сект./дорожка, %d дорожек, %d сторон, "
              "блок %d байт, каталог %d входов"
              % (drv.secsz, drv.spt, drv.tracks, drv.sides + 1, drv.alsize, drv.drm + 1))
        count = total = 0
        for entry in drv.files:
            data = b"".join(drv.read_file(entry["name"]))
            name = file_name(entry["name"])
            with open(os.path.join(outdir, name), "wb") as f:
                f.write(data)
            count += 1
            total += len(data)
            print("    %-13s %7d байт  записей %4d  блоков %3d"
                  % (name, len(data), entry["records"], len(entry["blocks"])))
        print("    итого: %d файлов, %d байт" % (count, total))
        return count, total
    finally:
        drv.close()


def main():
    args = parse_args()
    images = find_images(args.images)
    if not images:
        sys.exit("нет образов KDI (переданного файла нет или в каталоге пусто)")
    files = data = failed = 0
    for path in images:
        try:
            n, size = extract(path, args.o)
        except (ValueError, OSError) as e:
            failed += 1
            print("\n=== %s\n    - не читается: %s" % (path, e), file=sys.stderr)
            continue
        files += n
        data += size
    print("\nРаспаковано %d файлов (%d байт) из %d образов%s"
          % (files, data, len(images) - failed, ", с ошибками: %d" % failed if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
