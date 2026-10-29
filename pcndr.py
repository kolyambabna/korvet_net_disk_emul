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
#         + ветки записи в образы KDI: создание (22), приём секторов (21) с
#         проверкой КС, закрытие (16), удаление (19), переименование (23).
#         В C-версии эти функции только протоколировались в tempfile.bin.
#         + -yy N / -posty MS: одиночный байт 'Y' (или 'N') залипает в буфере
#         драйвера USB-RS232 (latency timer CH340/PL2303 ~16 мс) и не успевает
#         в таймаут клиента. Несколько байтов подряд драйвер выталкивает сразу.
#         Ответ монитора уходит до любой печати в консоли.
#

import argparse
import sys
import time

import serial

import common
from common import abort
from console import ConsoleKeys
from kdi import KdiDrive, KdiError
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
    p.add_argument("-yy", metavar="N", type=int, default=1,
                   help="сколько байтов подтверждения подряд отправлять (1 как в C); "
                        "одиночный 'Y' залипает в буфере драйвера USB-RS232, "
                        "на CH340/PL2303 помогает -yy 2 (по умолчанию %(default)s)")
    p.add_argument("-posty", metavar="MS", type=float, default=2.0,
                   help="пауза в миллисекундах после подтверждения, перед ответом "
                        "и следующим маркером опроса (по умолчанию %(default)s)")
    return p.parse_args()


def send_ack(ser, ch, count):
    """Подтверждение ('Y') или отказ ('N') в линию.

    Драйверы USB-RS232 (CH340, PL2303) держат одиночный байт до latency timer
    (~16 мс), и клиент не успевает его прочесть; несколько байтов подряд
    уходят сразу. flush обязателен: без него байт остаётся в буфере Python.
    """
    ser.write(ch * count)
    ser.flush()


def fmt_name(raw):
    """Подготовка к печати 11-байтного имени CP/M (аналог %.11s)."""
    return raw.split(b"\x00", 1)[0].decode("ascii", "replace")


def sel_drive(drives, drive):
    """Образ диска по номеру из пакета; None - такого диска нет."""
    if 0 <= drive < len(drives):
        return drives[drive]
    return None


def note(text):
    """Пояснение к операции - с новой строки, не затирая заголовок операции."""
    print("\n   %s" % text, end="", flush=True)


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
    if args.yy < 1:
        abort("\n - Число повторов подтверждения -yy должно быть не меньше 1\n")

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

    print(" + Подтверждение: %d байт(а), пауза после него %g мс"
          % (args.yy, args.posty), flush=True)
    if sys.platform == "win32" and args.yy == 1:
        # иначе одиночный 'Y' может не дойти до клиента за его таймаут
        print(" ! Если Корвет не видит ответы: поставьте Latency Timer=1 мс "
              "для %s или запускайте с -yy 2" % args.s, flush=True)

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
    wsector = 0     # счетчик принятых секторов записываемого файла
    wfile = None    # сессия записи в образ (KdiWriter), None - писать некуда
    wsum = 0        # нарастающая контрольная сумма принятых данных

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
                ser.flush()
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
                    # КС неправильная - сначала уходит 'N' и только потом печать,
                    # чтобы консоль не задерживала ответ в линии
                    send_ack(ser, b"N", args.yy)
                    print("\n - Пакет с неправильной контрольной суммой: %02x != %02x"
                          % (csum, pkt[33]), end="", flush=True)
                    continue
                send_ack(ser, b"Y", args.yy)  # КС правильная - подтверждаем пакет
                if args.posty > 0:
                    # пауза до данных/следующего маркера: клиенту надо успеть
                    # обработать подтверждение
                    time.sleep(args.posty / 1000.0)

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
                    drv = sel_drive(drives, drive)
                    if drv is None:
                        note("- диск %c не подключен" % dl)
                    elif drv.locate(raw_name) < 0:
                        note("- файла на образе нет")

                elif fn == 16:
                    cr_flag = 0
                    print("\n + Закрытие файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)
                    if wfile is None:
                        note("- писать было нечего, образ не изменен")
                    else:
                        try:
                            res = wfile.commit()   # данные и каталог уходят в образ
                        except KdiError as e:
                            note("- файл не записан: %s" % e)
                        else:
                            note("+ записано %d секторов (%d байт), блоков %d, "
                                 "свободно ещё %d"
                                 % (res["records"], res["records"] * 128,
                                    len(res["blocks"]),
                                    len(wfile.drv.free_blocks())))
                        wfile = None
                        wsum = 0

                elif fn == 19:
                    cr_flag = 0
                    print("\n + Удаление файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)
                    drv = sel_drive(drives, drive)
                    try:
                        if drv is None:
                            raise KdiError("диск %c не подключен" % dl)
                        n = drv.delete(raw_name)
                    except KdiError as e:
                        note("- %s" % e)
                    else:
                        note("+ помечено входов каталога: %d" % n)

                elif fn == 20:
                    # Последовательное чтение всего файла
                    cr_flag = 0
                    readfile(ser, drives, drive, raw_name, user)

                elif fn == 21:
                    # Последовательная запись файла: один 128-байтовый сектор + КС
                    ser.timeout = None  # ждем без таймаута весь блок из 129 байт
                    if cr_flag != 2:
                        print("\n", end="")
                        cr_flag = 2
                        wsector = 0
                    print("\r + Запись файла U%i: %c:%s  блок # %i"
                          % (user, dl, disp, wsector), end="", flush=True)
                    block = ser.read(129)  # принимаем блок
                    if len(block) < 129:
                        send_ack(ser, b"N", args.yy)
                        note("- принято %d байт вместо 129, запись прервана"
                             % len(block))
                        wfile = None
                    elif wfile is None:
                        # файл не создавался - принимать некуда
                        send_ack(ser, b"N", args.yy)
                        note("- файл не создавался, блок отклонен")
                    else:
                        # КС - нарастающая сумма байт файла, как при его чтении
                        want = (wsum + sum(block[:128])) & 0xFF
                        if want != block[128]:
                            send_ack(ser, b"N", args.yy)
                            note("- КС блока %02X != %02X, блок отброшен"
                                 % (block[128], want))
                        else:
                            # блок принят; при откате КС не накручиваем, чтобы
                            # повтор того же блока прошёл проверкой
                            wsum = want
                            wfile.add(block[:128])
                            wsector += 1
                            send_ack(ser, b"Y", args.yy)  # ответ клиенту

                elif fn == 22:
                    cr_flag = 0
                    print("\n + Создание файла U%i: %c:%s" % (user, dl, disp),
                          end="", flush=True)
                    wfile = None
                    wsum = 0
                    wsector = 0
                    drv = sel_drive(drives, drive)
                    try:
                        if drv is None:
                            raise KdiError("диск %c не подключен" % dl)
                        wfile = drv.open_write(raw_name)
                    except KdiError as e:
                        note("- создание не удалось: %s" % e)
                    else:
                        note("+ свободно блоков %d, входов каталога %d"
                             % (len(drv.free_blocks()), len(drv.free_slots())))

                elif fn == 23:
                    cr_flag = 0
                    new_raw = bytes(pkt[0x14:0x1F])
                    print("\n + Переименование файла U%i: %c:%s -> %s"
                          % (user, dl, disp, fmt_name(new_raw)),
                          end="", flush=True)
                    drv = sel_drive(drives, drive)
                    try:
                        if drv is None:
                            raise KdiError("диск %c не подключен" % dl)
                        n = drv.rename(raw_name, new_raw)
                    except KdiError as e:
                        note("- %s" % e)
                    else:
                        note("+ правок в каталоге: %d" % n)

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
        if wfile is not None:
            print("\n - Клиент не закрыл файл, образ не изменен")
        sysimage.close()
        for d in drives:
            if d is not None:
                d.close()
        ser.close()
        print()


if __name__ == "__main__":
    main()
