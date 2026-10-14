# -*- coding: utf-8 -*-
"""Процедуры работы с образом диска KDI (CP/N-90).

Python-аналог kdi.c сетевого монитора pcndr:
разбор информационного сектора, чтение каталога (с склейкой экстентов),
последовательное чтение файла в 128-байтовые логические сектора.
"""

from struct import unpack_from

SECTOR_SIZES = (128, 256, 512, 1024)  # размер физического сектора по коду из инфосектора


class KdiDrive:
    """Один образ диска KDI: параметры из инфосектора, каталог, чтение данных."""

    def __init__(self, path):
        self.path = path
        # открываем на чтение с записью, как в оригинале (запись не используется)
        self.file = open(path, "r+b")
        # Структура информационного сектора - смещения из inf оригинала
        hdr = self.file.read(0x21)
        if len(hdr) < 0x21:
            raise ValueError("образ KDI %s слишком мал" % path)
        self.count = unpack_from("<H", hdr, 0x04)[0]        # число загружаемых физических секторов
        self.secsz = SECTOR_SIZES[hdr[0x0A] & 0x03]         # размер физического сектора
        self.sides = hdr[0x0B]                              # 0 - односторонний, 1 - двухсторонний
        self.spt = unpack_from("<H", hdr, 0x0D)[0]          # физических секторов на дорожке
        self.tracks = unpack_from("<H", hdr, 0x0F)[0]       # дорожек на диске
        self.lspt = unpack_from("<H", hdr, 0x11)[0]         # логических секторов на дорожке
        self.bsh = hdr[0x13]                                # размер единицы распределения
        self.drm = unpack_from("<H", hdr, 0x18)[0]          # число входов в каталог минус 1
        self.systracks = unpack_from("<H", hdr, 0x1E)[0]    # число системных дорожек

        if self.lspt == 0 or self.spt == 0:
            raise ValueError("некорректные геометрия диска в инфосекторе %s" % path)
        self.blocking = (1 << self.bsh) & 0xFF              # число секторов в блоке (единице распределения)
        self.alsize = self.blocking * (self.spt * self.secsz // self.lspt)  # единица распределения, байт
        if self.alsize == 0:
            raise ValueError("нулевой размер единицы распределения в %s" % path)
        self.reserved = (self.count * self.secsz + (self.drm + 1) * 16) // self.alsize
        self.dirblocks = (self.drm + 1) * 16 // self.alsize
        self.files = []  # элементы каталога: {"name", "records", "blocks"}

    # ******************************************************
    # *  Позиционирование внутри образа на CHS
    # ******************************************************
    def seek_chs(self, cyl, head, sec):
        pos = (cyl * self.spt * (self.sides + 1) + head * self.spt + (sec - 1)) * self.secsz
        self.file.seek(pos)

    # ********************************************************
    # *  Поиск файла в массиве каталога
    # *  Возвращает индекс записи, -1 - не найден
    # ********************************************************
    def locate(self, name):
        name = bytes(name[:11])
        for i, f in enumerate(self.files):
            if f["name"] == name:
                return i
        return -1

    # ********************************************************
    # *  Чтение и разбор каталога диска.
    # *  Формирует массив self.files с разобранным содержимым.
    # ********************************************************
    def read_dir(self):
        sides1 = max(1, self.sides + 1)
        # встаем на начало каталога
        self.seek_chs(self.systracks // sides1, self.systracks % sides1, 1)
        self.files = []

        for _ in range(self.drm + 1):
            d = self.file.read(32)  # элемент каталога
            if len(d) < 32:
                break
            if d[0] == 0xE5:
                continue  # удаленные и неиспользованные элементы каталога
            name = bytes(b & 0x7F for b in d[1:12])  # имя и расширение
            ex = d[0x0C]                             # номер экстента
            rc = d[0x0F]                             # число записей в экстенте
            blocks = [b for b in unpack_from("<8H", d, 0x10) if b]
            if ex == 0:
                # Экстент 0 - создаем новую запись в массиве каталога
                self.files.append({"name": name, "records": rc, "blocks": blocks})
            else:
                # Дополнительные экстенты - добавляем к списку блоков
                i = self.locate(name)  # ищем запись начального экстента
                if i == -1:
                    print("\n Файл %s - нет начального экстента"
                          % name.decode("ascii", "replace"), end="", flush=True)
                    continue
                self.files[i]["records"] += rc
                self.files[i]["blocks"] += blocks

    # ******************************************************
    # *  Возвращает генератор 128-байтовых логических
    # *  секторов файла, либо None если файл не найден
    # ******************************************************
    def read_file(self, name):
        i = self.locate(name)
        if i < 0:
            return None
        return self._sectors(self.files[i])

    def _sectors(self, f):
        fsize = f["records"]  # размер файла в логических секторах
        # Цикл по блокам распределения
        for lba in f["blocks"]:
            # встаем на начало блока и читаем его
            self.file.seek((self.reserved + lba - self.dirblocks) * self.alsize)
            block = self.file.read(self.alsize)
            if len(block) < self.alsize:
                block += b"\xe5" * (self.alsize - len(block))
            # цикл по логическим секторам - пока счетчик секторов не дойдет до 0
            for j in range(self.blocking):
                yield block[j * 128:(j + 1) * 128]
                fsize -= 1
                if fsize <= 0:
                    return  # счетчик секторов закончился - выходим,
                            # даже если это середина блока распределения

    def close(self):
        self.file.close()
