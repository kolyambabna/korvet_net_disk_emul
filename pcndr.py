#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#   Программа эмуляции сервера сети CP/N-90 на PC
#
#   Python-порт сетевого монитора pcndr.
#   Автор оригинальной C-версии: Alexander Stepanov aka forth32 <forth32@mail.ru>
#
#  История изменений:
#
#  1.00 базовая версия (C)
#  1.01 добавлена поддержка диска E и сохранение записываемых файлов на диск (C)
#  1.01p Python-порт: кроссплатформенность (Linux/Windows), pyserial
#

import argparse
import sys

import serial

import common
from common import abort
from console import ConsoleKeys
from kdi import KdiDrive
from lan import Lan

VERSION = "1.01p"

SYSOFFSET = (0, 0, 0, 0, 2)  # число пропускаемых в образе KDI дорожек
                             # при доступе с разных букв диска (A..E)


def parse_args():
    p = argparse.ArgumentParser(
        prog="pcndr.py",
        description="Эмулятор сетевого сервера КОРВЕТ CP/N-90 (монитор NDR)")
    p.add_argument("-a", metavar="FILE", default="disk/sys.kdi",
                   help="имя файла образа диска A (по умолчанию %(default)s)")
    p.add_argument("-b", metavar="FILE", default="",
                   help="имя файла образа диска B")
    p.add_argument("-e", metavar="FILE", default="",
                   help="имя файла образа диска E")
    p.add_argument("-s", metavar="DEV", default="/dev/ttyS0",
                   help="имя последовательного устройства, например /dev/ttyUSB0 "
                        "или COM3 (по умолчанию %(default)s)")
    p.add_argument("-n", metavar="N", type=int, default=2,
                   help="адрес РМУ, по умолчанию %(default)s")
    p.add_argument("-x", action="store_true",
                   help="пропуск фазы загрузки системы")
    p.add_argument("-d", action="store_true",
                   help="отладочная печать сетевого обмена")
    return p.parse_args()


def fmt_name(raw):
    """Подготовка к печати 11-байтного имени CP/M (аналог %.11s)."""
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace")


def readfile(ser, drives, drive, raw_name, user):
    """Последовательное чтение и пересылка файла клиенту (аналог readfile из kdi.c)."""
    disp = fmt_name(raw_name)
    drv = drives[drive] if 0 <= drive < 5 else None
    sectors = drv.read_file(raw_name) if drv is not None else None
    if sectors is None:
        print("\n + Чтение файла %i: %s -  Файл не найден" % (user, disp),
              end="", flush=True)
        return
    print("\n", end="", flush=True)
    csum = 0
    sent = 0
    for sec in sectors:  # перекидываем сектора клиенту
        ser.write(sec)
        print("\r + Чтение файла U%i: %c:%s - сектор %i    "
              % (user, 65 + drive, disp, sent), end="", flush=True)
        csum = (csum + sum(sec)) & 0xFF
        sent += 1
    print(" - Пересылка завершена", end="", flush=True)
    ser.write(bytes([csum]))  # КС
    ser.flush()


def sector_read(ser, drives, sysimage, drive, track, sector):
    """Режим 0 - абсолютное чтение сектора диска и отсылка клиенту."""
    off = SYSOFFSET[drive] if 0 <= drive < 5 else 0
    print(" + Запрос сектора: диск %c дорожка %i сектор %i    "
          % (65 + drive if 0 <= drive < 26 else 63, track + off, sector),
          end="", flush=True)
    if drive == 0 and track < 2:
        # Теплый рестарт системы - подставляем образ системы из файла
        sysimage.seek(track * 5 * 1024 + (sector - 1) * 128)
        sbuf = sysimage.read(128)
    elif 0 <= drive < 5 and drives[drive] is not None:
        # просто читаем абсолютный сектор нужного нам диска
        drives[drive].file.seek((track + off) * 5 * 1024 + (sector - 1) * 128)
        sbuf = drives[drive].file.read(128)
    else:
        print(" - диск не определен, игнорируем", end="", flush=True)
        sbuf = b"\xe5" * 128
    if len(sbuf) < 128:  # образ короче запрошенного сектора
        sbuf += b"\xe5" * (128 - len(sbuf))
    csum = sum(sbuf) & 0xFF  # подсчет КС
    ser.write(sbuf)          # отсылаем блок
    ser.write(bytes([csum]))  # КС
    ser.flush()              # ждем полной отправки данных


def main():
    args = parse_args()
    if args.d:
        common.debug = 1
    nodeadr = args.n
    if not 1 <= nodeadr <= 15:
        abort("\n - Адрес РМУ должен быть в диапазоне 1-15\n")

    print("\n ***  Сетевой монитор КОРВЕТ CP/N-90 версия %s ***" % VERSION)

    # Настройка SIO: 19200 8O1, без аппаратного_handshake
    try:
        ser = serial.Serial(port=args.s, baudrate=19200,
                            bytesize=serial.EIGHTBITS,
                            stopbits=serial.STOPBITS_ONE,
                            parity=serial.PARITY_ODD,
                            xonxoff=False, rtscts=False, dsrdtr=False,
                            timeout=common.serial_timeout / 10.0)
    except serial.SerialException:
        abort("\n - Последовательный порт %s не открывается\n", args.s)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    marker = bytes([0xFF, 0x80 | nodeadr])  # маркер опроса клиента с адресом узла

    try:
        sysimage = open("cpn90sys.bin", "rb")
    except OSError:
        abort("\n - не найден файл образа системы cpn90sys.bin\n")

    # Открытие файлов образов KDI и чтение каталогов
    drives = [None] * 5
    for idx, letter, path in ((0, "A", args.a), (1, "B", args.b), (4, "E", args.e)):
        if not path:
            continue
        print("\n + Чтение каталога диска %s" % letter, end="", flush=True)
        try:
            drv = KdiDrive(path)
            drv.read_dir()
        except (OSError, ValueError):
            abort("\n - ошибка открытия образа диска %s - %s\n", letter, path)
        drives[idx] = drv

    lan = Lan(ser, nodeadr)

    # Отправка образа системы клиенту
    if not args.x:
        lan.send_system()

    print("\n + Ждем команду от клиента", end="", flush=True)

    cr_flag = 0     # флаг перехода на новую строку в логе операций
    wsector = 0     # счетчик записываемых секторов
    tempfile = None  # файл для записываемых данных

    try:
        with ConsoleKeys() as keys:
            while True:
                # =============== Проверка на консольные команды ===============
                ser.timeout = common.serial_timeout / 10.0
                cmd = keys.key()
                if cmd == "r":
                    # Перезагрузка ОС
                    lan.send_system()
                    print("\n + Ждем команду от клиента", end="", flush=True)
                    continue
                if cmd == "x":
                    print("\n + Выход из монитора")
                    break

                # ============ отправляем маркер опроса в сеть ============
                ser.write(marker)
                first = ser.read(1)
                if not first:
                    continue  # ответа нет
                # ответ получен - читаем побайтово командный пакет,
                # с таймаутом для предотвращения подвисаний
                pkt = bytearray(first)
                while len(pkt) < 34:
                    b = ser.read(1)
                    if not b:
                        break
                    pkt.append(b[0])
                if len(pkt) < 34:
                    pkt += bytes(34 - len(pkt))  # недочитанный пакет - заведомо бракуем по КС

                # ---------- Проверка контрольной суммы пакета ----------
                csum = sum(pkt[1:33]) & 0xFF
                if csum != pkt[33]:
                    # КС неправильная - ответ N
                    ser.write(b"N")
                    print("\n - Пакет с неправильной контрольной суммой: %02x != %02x"
                          % (csum, pkt[33]), end="", flush=True)
                    continue
                ser.write(b"Y")  # КС правильная - подтверждаем пакет

                # ---------- разбираем данные пакета ----------
                user = pkt[1] & 0x1F  # # user-области
                fn = pkt[2]           # код функции
                drive = pkt[32]
                raw_name = bytes(pkt[4:15])
                disp = fmt_name(raw_name)
                dl = chr(65 + drive) if 0 <= drive < 26 else "?"

                # ------- ветки обработчиков разных функций -------
                if fn == 0:
                    # Режим 0 - абсолютное чтение диска
                    drive, track, sector = pkt[3], pkt[4], pkt[5]
                    if cr_flag != 1:
                        print("\n", end="")
                        cr_flag = 1
                    else:
                        print("\r", end="")
                    sector_read(ser, drives, sysimage, drive, track, sector)

                elif fn == 15:
                    cr_flag = 0
                    print("\n + Открытие файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)

                elif fn == 16:
                    cr_flag = 0
                    print("\n + Закрытие файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)
                    if tempfile is not None:
                        tempfile.close()
                        tempfile = None

                elif fn == 19:
                    cr_flag = 0
                    print("\n + Удаление файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)

                elif fn == 20:
                    # Последовательное чтение всего файла
                    cr_flag = 0
                    readfile(ser, drives, drive, raw_name, user)

                elif fn == 21:
                    # Последовательная запись файла
                    ser.timeout = None  # ждем без таймаута весь блок из 129 байт
                    if cr_flag != 2:
                        print("\n", end="")
                        cr_flag = 2
                        wsector = 0
                    print("\r + Запись файла U%i: %c:%s  блок # %i"
                          % (user, dl, disp, wsector), end="", flush=True)
                    wsector += 1
                    block = ser.read(129)  # принимаем блок
                    ser.write(b"Y")        # ответ клиенту
                    if tempfile is not None:
                        tempfile.write(block[:128])

                elif fn == 22:
                    cr_flag = 0
                    print("\n + Создание файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)
                    if tempfile is not None:
                        tempfile.close()
                    tempfile = open("tempfile.bin", "wb")

                elif fn == 23:
                    cr_flag = 0
                    print("\n + Переименование файла U%i: %c:%s -> %s"
                          % (user, dl, disp, fmt_name(bytes(pkt[0x14:0x1F]))),
                          end="", flush=True)

                elif fn == 36:
                    cr_flag = 0
                    print("\n + SETRAN U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)

                else:
                    cr_flag = 0
                    print("\n - Неопределенная функция %i, файл U%i: %c:%s"
                          % (fn, user, dl, disp), end="", flush=True)
                sys.stdout.flush()

    except KeyboardInterrupt:
        print("\n + Выход из монитора")
    finally:
        if tempfile is not None:
            tempfile.close()
        sysimage.close()
        for d in drives:
            if d is not None:
                d.close()
        ser.close()
        print()


if __name__ == "__main__":
    main()
