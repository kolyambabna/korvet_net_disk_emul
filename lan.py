# -*- coding: utf-8 -*-
"""Процедуры работы со стандартной сетью ОПТС 2.0 (протокол CP/N-90).

Python-аналог std_lan.c сетевого монитора pcndr.
Класс Lan реализует «мастера» сети (монитор NDR) поверх последовательного
порта: адресные пакеты, пакеты данных с экранированием, загрузку системы.
"""

import time

from common import abort, dprintf

MAXLEN = 0x58        # максимальная длина пакета данных. Число 0x58 взято из ОПТС
ADR_DELAY = 0.04     # задержка перед посылкой адресного пакета. Нужна, чтобы все РМУ
                     # успели очухаться от предыдущих сеансов обмена
BYTE_DELAY = 0.0008  # пауза между байтами при отсылке пакета

SYNC = 0x01          # синхробайт начала пакета
CR = 0x0D            # байт конца пакета


class Lan:
    """Мастер сети ОПТС 2.0 поверх последовательного порта pyserial."""

    def __init__(self, ser, nodeadr):
        self.ser = ser              # открытый последовательный порт (объект pyserial)
        self.nodeadr = nodeadr      # адрес РМУ назначения
        self.pnum = 0               # последовательный номер отправляемого пакета
        self.ibuf = bytearray()     # буфер приема
        self.obuf = bytearray(257)  # буфер отправки

    # ****************************************
    # Вычисление контрольной суммы пакета *
    # ****************************************
    @staticmethod
    def checksum(buf, ln):
        csum = sum(buf[1:ln]) & 0xFF       # выделяем младший байт
        return (((csum >> 6) + csum) & 0x3F) + 0x20

    # ************************************************************
    # *  Чтение пакета из сети                                  *
    # *  Возвращает длину пакета,
    #    0  в случае таймаута
    #   -1 в случае неправильной КС
    # ************************************************************
    def readpkt(self):
        while True:
            # ждем синхробайта 1
            while True:
                c = self._read1()
                if c is None:
                    return 0
                if c == SYNC:
                    break
            ln = self._read1()
            if ln is None:
                return 0
            buf = bytearray([SYNC, ln])
            plen = (ln - 0x1D) & 0xFF      # реальная длина пакета, 2 байта уже отчитали
            complete = True
            while len(buf) < plen:
                c = self._read1()
                if c is None:
                    return 0               # таймаут - пакет не дочитан
                if c == SYNC:
                    complete = False       # новый синхробайт - перечитываем пакет
                    break
                buf.append(c)
            if complete:
                break
        self.ibuf = buf
        if plen >= 2 and self.checksum(buf, plen - 2) == buf[plen - 2]:
            return plen
        return -1

    # ****************************
    # *  Отсылка пакета в сеть   *
    # ****************************
    def sendpkt(self, buf, plen):
        buf[plen - 1] = CR                              # конец пакета
        buf[0] = SYNC                                   # синхробайт
        buf[1] = (plen + 0x1D) & 0xFF                   # длина пакета
        buf[plen - 2] = self.checksum(buf, plen - 2)    # контрольная сумма
        for i in range(plen):
            self.ser.write(bytes([buf[i]]))
            time.sleep(BYTE_DELAY)
        self.ser.flush()    # аналог tcdrain - ждем полной отправки данных

    # *****************************
    # *  Отсылка адресного пакета *
    # *
    # * num - # пакета
    # * mode - 0 - широковещательный
    # *        1 - адресный
    # * adr - адрес клиента
    # *****************************
    def pkt_a(self, pnum, mode, adr):
        o = self.obuf
        o[2] = pnum
        o[3] = ord('A')
        o[4] = ord('0') + mode
        o[5] = ord('1') if mode == 0 else ord("%01X" % adr)
        self.sendpkt(o, 8)

    # ********************************
    # *  Отсылка пакета конца данных *
    # ********************************
    def pkt_z(self, pnum):
        self.obuf[2] = pnum
        self.obuf[3] = ord('Z')
        self.sendpkt(self.obuf, 6)
        self.readpkt()

    # ****************************************
    # *  Отсылка пакета подтверждения данных *
    # ****************************************
    def pkt_y(self, pnum):
        self.obuf[2] = pnum
        self.obuf[3] = ord('Y')
        self.sendpkt(self.obuf, 6)

    # **********************************
    # *  Отсылка пакета ошибки данных  *
    # **********************************
    def pkt_n(self, pnum):
        self.obuf[2] = pnum
        self.obuf[3] = ord('N')
        self.sendpkt(self.obuf, 6)

    # ***********************************
    # *  Отсылка пакета передачи данных *
    # *
    # * pnum - # пакета
    # * mode - тип данных:
    #              0 - текстовое сообщение с предварительной очисткой экрана
    #              1 - образ GRAM
    #              2 - бейсик-программа
    #              3 - образ блока RAM
    #              5 - тело COM сообщения
    #              7 - образ блока RAM с передачей управления на начало блока
    #              9 - текстовое сообщение без очистки экрана
    # * adr - начальный адрес для образа памяти
    # ***********************************
    def pkt_r(self, pnum, mode, adr):
        o = self.obuf
        o[2] = pnum
        o[3] = ord('R')
        o[4] = ord('0') + mode
        o[5:9] = b"%04X" % adr
        self.sendpkt(o, 20)

    # ***********************************************************
    # *  Отсылка пакета D с упакованными данными                *
    # *  Возвращает фактическое число данных, помещенных в пакет *
    # ***********************************************************
    def pkt_d(self, pnum, dbuf, ln):
        o = self.obuf
        o[2] = pnum
        o[3] = ord('D')
        plen = 4  # 4 байта - заголовок пакета
        i = 0
        while i < ln:
            c = dbuf[i]
            cs = c & 0x80             # cs - старший бит
            c &= 0x7F                 # c - остальные биты кроме старшего
            if c == ord('#'):
                o[plen] = ord('#')    # удваиваем префиксный байт
                plen += 1
            elif c < 0x20 or c == 0x7F:
                o[plen] = ord('#')    # вводим префикс
                plen += 1
                c = (c + 0x40) & 0x7F
            o[plen] = c | cs
            plen += 1
            i += 1
            if plen >= MAXLEN:
                break
        self.sendpkt(o, plen + 2)
        return i

    # **********************************************
    # *   Посылка адреса с проверкой подтверждения *
    # *  Возвращает 1 если ответ получен
    # *  или 0 если ответ неправильный или не получен
    # **********************************************
    def send_adr(self, adr):
        errcount = 10
        self.pnum = 0x20  # начинаем счет пакетов сначала
        while True:
            time.sleep(ADR_DELAY)
            dprintf("\n @ Посылка адреса: ")
            self.pkt_a(self.pnum, 1, adr)
            self.pnum += 1
            res = self.readpkt()
            if res == 0:
                dprintf(" таймаут ")
                return 0
            if len(self.ibuf) > 3 and self.ibuf[3] == ord('Y'):
                dprintf(" OK")
                return 1
            if res == -1:
                dprintf(" ошибка контрольной суммы")
            elif len(self.ibuf) > 3:
                dprintf(" подтверждение не получено - %s", chr(self.ibuf[3] & 0x7F))
            errcount -= 1
            if errcount == 0:
                return 0

    # ***************************************************
    # * Отправка массива данных в сеть
    # *  Возвращает количество переданных байт
    # *  0 - данные не переданы
    # ***************************************************
    def send_data(self, dbuf, dsize):
        dprintf("\n + Передача данных:  ")
        dptr = 0
        while dptr < dsize:
            # цикл попыток отослать пакет
            errcount = 0
            while True:
                ds = self.pkt_d(self.pnum, dbuf[dptr:], dsize - dptr)
                self.readpkt()
                if len(self.ibuf) > 3 and self.ibuf[3] == ord('Y'):
                    break
                errcount += 1
                if len(self.ibuf) > 3:
                    print(chr(self.ibuf[3]), end="", flush=True)
                if errcount > 10:
                    dprintf("\n - Превышено число попыток передачи")
                    return 0
            dptr += ds
            self.pnum += 1
            if self.pnum == 0x60:
                self.pnum = 0x20
            print(".", end="", flush=True)
        self.pkt_z(self.pnum)
        self.pnum += 1
        dprintf("\n + Передано %d байт", dptr)
        if dsize != 0:
            return dptr   # если передавались хоть какие-то данные
        return 1          # если данных не передавалось - случай запуска с адреса,
                          # для обхода контроля ошибок

    # ************************************
    # *   Запрос и передача данных к РМУ *
    # *  0 - запрос отклонен
    # *  !=0 - количество переданных байт
    # ************************************
    def putdata(self, dbuf, typ, start, dsize):
        if self.send_adr(self.nodeadr) == 0:
            return 0
        self.pkt_r(self.pnum, typ, start)  # команда отправки данных
        self.pnum += 1
        dprintf("\n @ Послана команда отправки данных")
        self.readpkt()  # получаем пакет подтверждения Y
        if len(self.ibuf) < 4 or self.ibuf[3] != ord('Y'):
            dprintf("\n - Подтверждение не получено")
            return 0
        # отправка данных
        return self.send_data(dbuf, dsize)

    # ************************************
    # *   Вывод сообщения на консоль РМУ *
    # ************************************
    def send_message(self, text, xclean=False):
        data = text.encode("cp866", "replace")
        # 0 - с очисткой экрана, 9 - без очистки
        self.putdata(data, 0 if xclean else 9, 0, len(data))

    # ***********************************************
    # * Отправка загрузчика и образа системы на РМУ
    # ***********************************************
    def send_system(self):
        progress = "-\\-/"

        try:
            with open("cpn90ldr.bin", "rb") as image:
                buf = image.read(1024)
        except OSError:
            abort("\n - Не найден файл загрузчика системы cpm90ldr.bin\n")

        print("\n + Посылка загрузчика", flush=True)
        self.send_message(" --------- BOOT ----------------", True)
        # тип 7 - образ RAM с передачей управления на начало блока
        if self.putdata(buf, 7, 0xEEEE, len(buf)) == 0:
            abort("\n - Ошибка передачи загрузчика\n")

        self.ser.flush()
        time.sleep(0.2)  # задержка на время инициализации загрузчика в корвете

        try:
            image = open("cpn90img.bin", "rb")
        except OSError:
            abort("\n - Не найден файл образа системы cpn90img.bin\n")

        # Перекидываем клиенту весь образ системы по 128 байт
        with image:
            print("\n + Посылка образа системы:  ", end="", flush=True)
            for i in range(88):
                block = image.read(128)
                if not block:
                    break
                self.ser.write(block)
                print("\b" + progress[i & 3], end="", flush=True)
                self.ser.flush()
            print("\n + Загрузка системы окончена", flush=True)

    # ------------------------------------------------------------------
    def _read1(self):
        """Чтение одного байта с текущим таймаутом порта. None - таймаут."""
        b = self.ser.read(1)
        return b[0] if b else None
