# -*- coding: utf-8 -*-
"""Процедуры работы с образом диска KDI (CP/N-90).

Python-аналог kdi.c сетевого монитора pcndr:
разбор информационного сектора, чтение каталога (с склейкой экстентов),
последовательное чтение файла в 128-байтовые логические сектора.

Запись файлов добавлена сверх kdi.c (в C-версии её нет): файл целиком копится в
памяти и только в commit() одним проходом кладётся в образ вместе с описателями
каталога, туда же выделяются блоки распределения. Так прерванная передача не
трогает образ. Ошибки файловых операций поднимают KdiError.
"""

from struct import pack, unpack_from

SECTOR_SIZES = (128, 256, 512, 1024)  # размер физического сектора по коду из инфосектора
ENTRY_SIZE = 32          # размер одного описателя каталога CP/M
DELETED = 0xE5           # номер user-области у удалённого (свободного) описателя
BLOCK_SLOTS = 8          # номеров блоков распределения в описателе (по 16 бит)


class KdiError(Exception):
    """Ошибка файловой операции с образом KDI - нет файла, нет места, плохое имя."""


def fcb_name(name):
    """Нормализовать имя файла к FCB CP/M: 8 байт имени + 3 расширения."""
    raw = bytes(bytes(name[:11]).ljust(11))
    clean = bytes(b & 0x7F for b in raw)      # старшие биты байтов имени - атрибуты
    if any(c < 0x20 or c == 0x7F for c in clean):
        raise KdiError("недопустимые символы в имени %r" % (name,))
    if not clean.strip():
        raise KdiError("пустое имя файла")
    return clean


def disp(name):
    """Имя FCB для сообщения в консоли."""
    return fcb_name(name).decode("ascii")


class KdiDrive:
    """Один образ диска KDI: параметры из инфосектора, каталог, чтение и запись данных."""

    def __init__(self, path):
        self.path = path
        # открываем на чтение с записью, как в оригинале; образ меняют только
        # операции записи этого модуля (commit/delete/rename)
        self.file = open(path, "r+b")
        try:
            self._read_info()
        except ValueError:
            self.file.close()   # битый образ не оставляем с открытым дескриптором
            raise
        self.files = []  # элементы каталога: {"name", "records", "blocks"}

    # ********************************************************
    # *  Разбор информационного сектора и производных параметров
    # ********************************************************
    def _read_info(self):
        # Структура информационного сектора - packed-структура inf[] из kdi.c.
        # Смещения взяты из реальной раскладки структуры (0x20 байт), а не из
        # комментариев kdi.c, где начиная с spt все адреса завышены на 1.
        hdr = self.file.read(0x20)
        if len(hdr) < 0x20:
            raise ValueError("образ KDI %s слишком мал" % self.path)
        self.count = unpack_from("<H", hdr, 0x04)[0]        # число загружаемых физических секторов
        self.secsz = SECTOR_SIZES[hdr[0x0A] & 0x03]         # размер физического сектора
        self.sides = hdr[0x0B]                              # 0 - односторонний, 1 - двухсторонний
        self.spt = unpack_from("<H", hdr, 0x0C)[0]          # физических секторов на дорожке
        self.tracks = unpack_from("<H", hdr, 0x0E)[0]       # дорожек на диске
        self.lspt = unpack_from("<H", hdr, 0x10)[0]         # логических секторов на дорожке
        self.bsh = hdr[0x12]                                # размер единицы распределения
        self.dsm = unpack_from("<H", hdr, 0x15)[0]          # последний номер блока распределения
        self.drm = unpack_from("<H", hdr, 0x17)[0]          # число входов в каталог минус 1
        self.systracks = unpack_from("<H", hdr, 0x1D)[0]    # число системных дорожек

        if self.lspt == 0 or self.spt == 0:
            raise ValueError("некорректная геометрия диска в инфосекторе %s" % self.path)
        self.blocking = (1 << self.bsh) & 0xFF              # число секторов в блоке (единице распределения)
        self.alsize = self.blocking * (self.spt * self.secsz // self.lspt)  # единица распределения, байт
        if self.alsize == 0:
            raise ValueError("нулевой размер единицы распределения в %s" % self.path)
        self.reserved = (self.count * self.secsz + (self.drm + 1) * 16) // self.alsize
        self.dirblocks = (self.drm + 1) * 16 // self.alsize
        # Начало каталога - граница CHS сразу за системными дорожками; это тот же
        # адрес, что seek_chs(systracks // (sides+1), systracks % (sides+1), 1).
        self.dirpos = self.systracks * self.spt * self.secsz

    # ******************************************************
    # *  Позиционирование внутри образа на CHS
    # ******************************************************
    def seek_chs(self, cyl, head, sec):
        pos = (cyl * self.spt * (self.sides + 1) + head * self.spt + (sec - 1)) * self.secsz
        self.file.seek(pos)

    def block_address(self, lba):
        """Смещение блока распределения lba в образе."""
        return (self.reserved + lba - self.dirblocks) * self.alsize

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
    # *  Каталог как последовательность 32-байтовых описателей
    # ********************************************************
    def entries(self):
        """Генератор каталога: (номер входа, описатель из 32 байт)."""
        self.file.seek(self.dirpos)     # встаем на начало каталога
        for n in range(self.drm + 1):
            d = self.file.read(ENTRY_SIZE)
            if len(d) < ENTRY_SIZE:
                return                  # образ кончился внутри каталога
            yield n, bytearray(d)

    def entry_at(self, n):
        """Описатель n-го входа каталога, None - если его нет в образе."""
        self.file.seek(self.dirpos + ENTRY_SIZE * n)
        d = self.file.read(ENTRY_SIZE)
        return bytearray(d) if len(d) == ENTRY_SIZE else None

    def entry_positions(self, name):
        """Номера всех входов каталога файла name (каждый экстент отдельно)."""
        name = bytes(name[:11])
        return [n for n, d in self.entries()
                if d[0] != DELETED and bytes(b & 0x7F for b in d[1:12]) == name]

    def _set_entry(self, n, data):
        self.file.seek(self.dirpos + ENTRY_SIZE * n)
        self.file.write(bytes(data))

    # ********************************************************
    # *  Чтение и разбор каталога диска.
    # *  Формирует массив self.files с разобранным содержимым.
    # ********************************************************
    def read_dir(self):
        self.files = []

        for _n, d in self.entries():
            if d[0] == DELETED:
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
            self.file.seek(self.block_address(lba))
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

    # ********************************************************
    # *  Карта выделения блоков распределения
    # ********************************************************
    def used_blocks(self, except_name=None):
        """Номера занятых блоков; except_name - блоки этого файла не учитывать."""
        skip = fcb_name(except_name) if except_name is not None else None
        used = set()
        for _n, d in self.entries():
            if d[0] == DELETED:
                continue
            if skip is not None and bytes(b & 0x7F for b in d[1:12]) == skip:
                continue
            used.update(b for b in unpack_from("<8H", d, 0x10) if b)
        return used

    def first_block(self):
        """Первый номер блока, который можно отдать файлу.

        Формула инфосектора считает каталог в (drm+1)*16 байт (потому и dirblocks
        занижен), а по факту каталог занимает (drm+1)*32, то есть на демо-диске
        два блока вместо одного. Нижняя граница выделения считается по настоящему
        концу каталога, чтобы файлы не накладывались на него.
        """
        dir_end = self.dirpos + (self.drm + 1) * ENTRY_SIZE
        return max(1, -(-dir_end // self.alsize) + self.dirblocks - self.reserved)

    def max_block(self):
        """Последний возможный номер блока: меньшее из dsm и конца образа."""
        phys = self.file.seek(0, 2) // self.alsize - 1 - (self.reserved - self.dirblocks)
        return min(self.dsm, phys) if self.dsm else phys

    def free_blocks(self, except_name=None):
        """Свободные номера блоков по возрастанию."""
        used = self.used_blocks(except_name)
        return [b for b in range(self.first_block(), self.max_block() + 1)
                if b not in used]

    def free_slots(self, except_name=None):
        """Номера свободных входов каталога; except_name - считать освобождаемыми."""
        free = [n for n, d in self.entries() if d[0] == DELETED or not any(d)]
        if except_name is not None:
            free.extend(self.entry_positions(except_name))
        return sorted(set(free))

    # ********************************************************
    # *  Запись файла: данные копятся в памяти, в образ
    # *  попадают только в KdiWriter.commit()
    # ********************************************************
    def open_write(self, name):
        """Начать запись файла; прежний файл с этим именем будет заменён."""
        return KdiWriter(self, name)

    # ********************************************************
    # *  Удаление файла: все его описатели помечаются удалёнными,
    # *  блоки распределения становятся свободными
    # ********************************************************
    def delete(self, name):
        positions = self.entry_positions(name)
        if not positions:
            raise KdiError("файл %s на образе не найден" % disp(name))
        for n in positions:
            e = self.entry_at(n)
            e[0] = DELETED
            self._set_entry(n, e)
        self.file.flush()
        self.read_dir()
        return len(positions)

    # ********************************************************
    # *  Переименование файла - имя правится во всех его экстентах
    # ********************************************************
    def rename(self, name, newname):
        new = fcb_name(newname)
        if self.entry_positions(new):
            raise KdiError("файл %s на образе уже есть" % disp(new))
        positions = self.entry_positions(name)
        if not positions:
            raise KdiError("файл %s на образе не найден" % disp(name))
        for n in positions:
            e = self.entry_at(n)
            e[1:12] = new
            self._set_entry(n, e)
        self.file.flush()
        self.read_dir()
        return len(positions)

    def close(self):
        self.file.close()


class KdiWriter:
    """Последовательная запись одного файла в образ KDI.

    Монитор принимает файл по 128-байтовой записи (fn=21 протокола ОПТС), поэтому
    записи копятся в памяти, а в момент commit() выделяются блоки, пишутся данные
    и описатели каталога. Прерванная передача (без commit) образ не меняет.
    """

    def __init__(self, drv, name):
        self.drv = drv
        self.name = fcb_name(name)
        self.data = bytearray()
        # прежний файл с таким именем: его блоки и входы каталога освобождаем при commit
        self.replaced = drv.entry_positions(self.name)

    @property
    def records(self):
        """Число принятых 128-байтовых записей."""
        return len(self.data) // 128

    def add(self, record):
        """Принять одну запись CP/M (128 байт); короткий кусок дополняем нулями."""
        self.data += bytes(record[:128]).ljust(128, b"\x00")

    def extents(self):
        """Размер плана раскладки: (описателей каталога, блоков, записей в экстенте)."""
        per = min(BLOCK_SLOTS, 255 // self.drv.blocking) * self.drv.blocking
        records = self.records
        return max(1, -(-records // per)), -(-records // self.drv.blocking), per

    def commit(self):
        """Выделяет блоки, пишет данные и каталог. Возвращает {"name","records","blocks"}."""
        drv = self.drv
        records = self.records
        nent, nblocks, per = self.extents()
        slots_in_ext = per // drv.blocking

        free = drv.free_blocks(except_name=self.name)
        if len(free) < nblocks:
            raise KdiError("не хватает места на образе: блоков нужно %d, свободно %d"
                           % (nblocks, len(free)))
        slots = drv.free_slots(except_name=self.name)
        if len(slots) < nent:
            raise KdiError("не хватает места в каталоге: входов нужно %d, свободно %d"
                           % (nent, len(slots)))
        blocks = free[:nblocks]

        # прежий файл помечаем удалённым - его блоки освобождаются
        for n in self.replaced:
            e = drv.entry_at(n)
            e[0] = DELETED
            drv._set_entry(n, e)

        for i, lba in enumerate(blocks):
            chunk = self.data[i * drv.alsize:(i + 1) * drv.alsize]
            drv.file.seek(drv.block_address(lba))
            drv.file.write(chunk)

        for i in range(nent):
            e = bytearray(ENTRY_SIZE)
            e[0] = 0                              # user-область: монитор работает под нулевой
            e[1:12] = self.name
            e[0x0C] = i                           # номер экстента
            e[0x0F] = min(per, records - i * per)  # записей в этом экстенте
            nums = (blocks[i * slots_in_ext:(i + 1) * slots_in_ext]
                    + [0] * BLOCK_SLOTS)[:BLOCK_SLOTS]
            e[0x10:0x20] = pack("<8H", *nums)
            drv._set_entry(slots[i], e)

        drv.file.flush()
        drv.read_dir()
        return {"name": self.name, "records": records, "blocks": blocks}
