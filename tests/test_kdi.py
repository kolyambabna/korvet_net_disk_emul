# -*- coding: utf-8 -*-
"""Тесты читателя образов KDI (kdi.py): синтетические образы + демо-диск disk/sys.kdi.

Синтетические образы собираются по формулам из docs/KDI.md (§2) — независимо от
реализации, чтобы сверка содержимого была самостоятельной проверкой, а не копией
кода kdi.py. Смещения инфосектора соответствуют packed-структуре inf[] из kdi.c
(её размер — 0x20 байт); комментарии в kdi.c начиная с spt завышены на 1 байт.

Железо и pyserial не нужны. Запуск из корня проекта:
    python -m unittest discover -s py/tests -v
или напрямую:
    python py/tests/test_kdi.py -v
"""

import os
import shutil
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from kdi import SECTOR_SIZES, KdiDrive, KdiError  # noqa: E402

SAMPLE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "disk", "sys.kdi"))


def fcb(base, ext=b"   "):
    """Имя файла в формате FCB CP/M: 8 байт имени + 3 расширения, пробелы вместо точки."""
    return (base + b" " * (8 - len(base))) + ext


def dentry(user, name, ex, rc, blocks):
    """32-байтовый описатель файла в каталоге CP/M."""
    assert len(name) == 11, "имя должно быть длиной 11 байт (FCB)"
    e = bytearray(32)
    e[0] = user
    e[1:12] = name
    e[0x0C] = ex
    e[0x0F] = rc
    nums = list(blocks[:8]) + [0] * (8 - min(8, len(blocks)))
    e[0x10:0x20] = struct.pack("<8H", *nums)
    return e


def info_sector(ssize=0, sides=0, spt=16, tracks=40, lspt=16, bsh=4, drm=63,
                systracks=2, count=None, dsm=0):
    """Инфосектор по раскладке packed-структуры inf[] из kdi.c (0x20 байт)."""
    h = bytearray(0x20)
    struct.pack_into("<H", h, 0x00, 0x0100)          # LoadAdress
    struct.pack_into("<H", h, 0x02, 0x0100)          # RunAdress
    struct.pack_into("<H", h, 0x04, spt * systracks if count is None else count)
    h[0x06] = 0                                      # диск 5.25"
    h[0x07] = 1                                      # MFM
    h[0x0A] = ssize
    h[0x0B] = sides
    struct.pack_into("<H", h, 0x0C, spt)
    struct.pack_into("<H", h, 0x0E, tracks)
    struct.pack_into("<H", h, 0x10, lspt)
    h[0x12] = bsh
    h[0x13] = (1 << bsh) - 1                         # blm
    struct.pack_into("<H", h, 0x15, dsm)             # последний номер блока
    struct.pack_into("<H", h, 0x17, drm)
    struct.pack_into("<H", h, 0x1D, systracks)
    return h


class KdiImage:
    """Сборка синтетического образа KDI: инфосектор, системная область, каталог, блоки.

    Каталог стоит на CHS-границе сразу за системными дорожками, блок распределения
    lba — в (reserved + lba - dirblocks) * alsize (docs/KDI.md, §2 и §4).
    """

    def __init__(self, ssize=0, sides=0, spt=16, tracks=40, lspt=16, bsh=4,
                 drm=63, systracks=2, dsm=0):
        self.ssize = ssize
        self.sides = sides
        self.spt = spt
        self.tracks = tracks
        self.lspt = lspt
        self.bsh = bsh
        self.drm = drm
        self.systracks = systracks
        self.dsm = dsm
        self.secsz = SECTOR_SIZES[ssize & 3]
        self.blocking = (1 << bsh) & 0xFF
        self.alsize = self.blocking * (spt * self.secsz // lspt) if lspt else 0
        self.count = spt * systracks                 # физических секторов до каталога
        if self.alsize:
            self.reserved = (self.count * self.secsz + (drm + 1) * 16) // self.alsize
            self.dirblocks = (drm + 1) * 16 // self.alsize
        else:
            self.reserved = self.dirblocks = 0
        self.dirpos = systracks * spt * self.secsz   # он же seek_chs после системных
        self.entries = []
        self.blocks = {}

    def add(self, base, rc, blocks, ext=b"   ", ex=0, user=0):
        self.entries.append(dentry(user, fcb(base, ext), ex, rc, blocks))
        return self

    def block(self, lba, payload):
        """Содержимое блока данных lba (дополняется нулями до alsize)."""
        self.blocks[lba] = bytes(payload[:self.alsize]).ljust(self.alsize, b"\x00")
        return self

    def block_offset(self, lba):
        return (self.reserved + lba - self.dirblocks) * self.alsize

    def build(self, trim=0, full=False):
        """Собрать образ. full=True — добить до полного размера по CHS,
        trim>0 — оборвать образ на trim байт раньше конца."""
        hdr = info_sector(ssize=self.ssize, sides=self.sides, spt=self.spt,
                          tracks=self.tracks, lspt=self.lspt, bsh=self.bsh,
                          drm=self.drm, systracks=self.systracks, count=self.count,
                          dsm=self.dsm)
        size = self.dirpos + (self.drm + 1) * 32
        for lba, data in self.blocks.items():
            size = max(size, self.block_offset(lba) + len(data))
        if full:
            size = max(size, self.spt * (self.sides + 1) * self.tracks * self.secsz)
        img = bytearray(size)
        img[:len(hdr)] = hdr
        empty = dentry(0xE5, fcb(b""), 0, 0, [])
        for n in range(self.drm + 1):
            src = self.entries[n] if n < len(self.entries) else empty
            img[self.dirpos + 32 * n:self.dirpos + 32 * n + 32] = src
        for lba, data in self.blocks.items():
            off = self.block_offset(lba)
            img[off:off + len(data)] = data
        return bytes(img[:len(img) - trim]) if trim else bytes(img)


def _unlink(path):
    try:
        os.unlink(path)
    except OSError:
        pass  # образ ещё открыт (тесты битого инфосектора) — уберёт временный каталог


class TempImageTestCase(unittest.TestCase):
    """Временные образы: дескрипторы закрываются, файлы удаляются в tearDown."""

    def setUp(self):
        self.drives = []
        self.paths = []

    def tearDown(self):
        for drv in self.drives:
            drv.close()
        for path in self.paths:
            _unlink(path)

    def drive(self, image):
        """Записать образ во временный файл и вернуть открытый KdiDrive."""
        fd, path = tempfile.mkstemp(suffix=".kdi")
        with os.fdopen(fd, "wb") as f:
            f.write(image)
        self.paths.append(path)
        drv = KdiDrive(path)
        self.drives.append(drv)
        return drv

    def raw(self, drv):
        with open(drv.path, "rb") as f:
            return f.read()


class InfoSectorTest(TempImageTestCase):
    """Разбор инфосектора и производных параметров (docs/KDI.md, §1—2)."""

    # Значения полей нарочно все разные: сдвиг любого смещения ломает разбор.
    GEOM = dict(ssize=2, sides=1, spt=10, tracks=64, lspt=40, bsh=4, drm=127,
                systracks=2)

    def test_fields(self):
        drv = self.drive(KdiImage(**self.GEOM).build())
        self.assertEqual(drv.count, 20)          # spt * systracks
        self.assertEqual(drv.secsz, 512)
        self.assertEqual(drv.sides, 1)
        self.assertEqual(drv.spt, 10)
        self.assertEqual(drv.tracks, 64)
        self.assertEqual(drv.lspt, 40)
        self.assertEqual(drv.bsh, 4)
        self.assertEqual(drv.drm, 127)
        self.assertEqual(drv.systracks, 2)

    def test_derived(self):
        drv = self.drive(KdiImage(**self.GEOM).build())
        self.assertEqual(drv.blocking, 16)       # 1 << bsh
        self.assertEqual(drv.alsize, 2048)       # blocking * (spt * secsz / lspt)
        self.assertEqual(drv.reserved, 6)        # (count*secsz + (drm+1)*16) // alsize
        self.assertEqual(drv.dirblocks, 1)       # (drm+1)*16 // alsize
        # каталог занимает ровно один блок, область данных начинается сразу за ним
        self.assertEqual(drv.systracks * drv.spt * drv.secsz,
                         (drv.reserved - drv.dirblocks) * drv.alsize)

    def test_sector_size_codes(self):
        for code, secsz in enumerate(SECTOR_SIZES):
            with self.subTest(code=code, secsz=secsz):
                spt = {128: 16, 256: 8, 512: 10, 1024: 5}[secsz]
                img = KdiImage(ssize=code, spt=spt, lspt=spt * secsz // 128)
                drv = self.drive(img.build())
                self.assertEqual(drv.secsz, secsz)
                self.assertEqual(drv.alsize, 2048)   # логический сектор всегда 128

    def test_sector_size_code_masked(self):
        # размер задаётся младшими двумя битами, как в оригинале: 0b101 -> 256
        drv = self.drive(KdiImage(ssize=0x05).build())
        self.assertEqual(drv.secsz, 256)


class SeekChsTest(TempImageTestCase):
    """Позиционирование внутри образа на CHS (аналог pchs_seek)."""

    def setUp(self):
        super().setUp()
        self.img = KdiImage(ssize=2, sides=1, spt=10, tracks=64, lspt=40, drm=127)

    def test_offsets(self):
        drv = self.drive(self.img.build(full=True))
        size = len(self.raw(drv))
        for cyl, head, sec in ((0, 0, 1), (0, 1, 10), (1, 0, 1), (37, 1, 7),
                               (63, 1, 10)):
            with self.subTest(cyl=cyl, head=head, sec=sec):
                drv.seek_chs(cyl, head, sec)
                want = (cyl * 10 * 2 + head * 10 + (sec - 1)) * 512
                self.assertEqual(drv.file.tell(), want)
                self.assertLessEqual(want + 512, size)

    def test_single_sided_never_skips_head(self):
        drv = self.drive(KdiImage(spt=16, tracks=40, drm=63).build(full=True))
        drv.seek_chs(3, 0, 5)
        self.assertEqual(drv.file.tell(), (3 * 16 + 0 + 4) * 128)

    def test_seek_reaches_written_bytes(self):
        marker = b"KDI-MARKER"
        off = (63 * 10 * 2 + 1 * 10) * 512         # дорожка cyl=63, head=1, сектор 1
        img = bytearray(self.img.build()).ljust(off + len(marker), b"\x00")
        img[off:off + len(marker)] = marker
        drv = self.drive(bytes(img))
        drv.seek_chs(63, 1, 1)
        self.assertEqual(drv.file.read(len(marker)), marker)


class ReadDirTest(TempImageTestCase):
    """Чтение и разбор каталога: экстенты, удалённые входы, атрибуты, обрыв."""

    def setUp(self):
        super().setUp()
        self.img = KdiImage()

    def read_dir(self, img=None):
        drv = self.drive((img or self.img).build())
        drv.read_dir()
        return drv

    def test_single_extent(self):
        self.img.add(b"A", 3, [1], ext=b"COM")
        drv = self.read_dir()
        self.assertEqual(len(drv.files), 1)
        self.assertEqual(drv.files[0], {"name": fcb(b"A", b"COM"), "records": 3,
                                        "blocks": [1]})

    def test_extents_are_glued(self):
        self.img.add(b"B", 16, [1, 2], ext=b"DOC", ex=0)
        self.img.add(b"B", 9, [3], ext=b"DOC", ex=1)
        drv = self.read_dir()
        self.assertEqual(len(drv.files), 1)
        self.assertEqual(drv.files[0]["records"], 25)       # 16 + 9 из двух экстентов
        self.assertEqual(drv.files[0]["blocks"], [1, 2, 3])

    def test_interleaved_extents(self):
        self.img.add(b"A", 16, [1], ext=b"COM", ex=0)
        self.img.add(b"B", 20, [5, 6], ext=b"COM", ex=0)
        self.img.add(b"A", 4, [2], ext=b"COM", ex=1)
        self.img.add(b"B", 7, [7], ext=b"COM", ex=1)
        drv = self.read_dir()
        self.assertEqual([f["name"] for f in drv.files],
                         [fcb(b"A", b"COM"), fcb(b"B", b"COM")])
        self.assertEqual(drv.files[0]["blocks"], [1, 2])
        self.assertEqual(drv.files[1]["blocks"], [5, 6, 7])
        self.assertEqual([f["records"] for f in drv.files], [20, 27])

    def test_deleted_entries_skipped(self):
        self.img.add(b"A", 1, [1], ext=b"COM")
        self.img.entries.append(dentry(0xE5, fcb(b"GONE", b"COM"), 0, 9, [9]))
        self.img.add(b"B", 2, [2], ext=b"COM")
        drv = self.read_dir()
        self.assertEqual([f["name"] for f in drv.files],
                         [fcb(b"A", b"COM"), fcb(b"B", b"COM")])

    def test_attribute_bits_masked(self):
        # старшие биты байтов имени — атрибуты CP/M, в массиве остаётся & 0x7F
        raw = bytearray(fcb(b"PIN"))
        raw[0] |= 0x80                                     # 'P' со старшим битом-атрибутом
        self.img.entries.append(dentry(0, bytes(raw), 0, 1, [1]))
        drv = self.read_dir()
        self.assertEqual(drv.files[0]["name"], fcb(b"PIN"))
        self.assertEqual(len(list(drv.read_file(fcb(b"PIN")))), 1)

    def test_orphan_extent_reported_and_skipped(self):
        self.img.add(b"LONE", 5, [3], ext=b"COM", ex=2)     # нет экстента 0
        out = StringIO()
        with redirect_stdout(out):
            drv = self.read_dir()
        self.assertEqual(drv.files, [])
        self.assertIn("нет начального экстента", out.getvalue())
        self.assertIn("LONE", out.getvalue())

    def test_read_dir_is_repeatable(self):
        self.img.add(b"A", 1, [1], ext=b"COM")
        drv = self.drive(self.img.build())
        drv.read_dir()
        drv.read_dir()
        self.assertEqual([f["name"] for f in drv.files], [fcb(b"A", b"COM")])

    def test_directory_cut_by_end_of_image(self):
        # образ оборван внутри каталога: разбираются только целые описатели
        img = (KdiImage().add(b"A", 1, [1], ext=b"COM")
                             .add(b"B", 1, [2], ext=b"COM")
                             .add(b"C", 1, [3], ext=b"COM"))
        rest = (img.drm + 1) * 32 - (2 * 32 + 7)  # остаются два целых входа + огрызок
        drv = self.drive(img.build(trim=rest))
        drv.read_dir()
        self.assertEqual([f["name"] for f in drv.files],
                         [fcb(b"A", b"COM"), fcb(b"B", b"COM")])


class LocateTest(TempImageTestCase):
    """Поиск файла в массиве каталога."""

    def setUp(self):
        super().setUp()
        img = KdiImage().add(b"A", 1, [1], ext=b"COM").add(b"B", 2, [2], ext=b"DOC")
        self.drv = self.drive(img.build())
        self.drv.read_dir()

    def test_found_indices(self):
        self.assertEqual(self.drv.locate(fcb(b"A", b"COM")), 0)
        self.assertEqual(self.drv.locate(fcb(b"B", b"DOC")), 1)

    def test_not_found(self):
        self.assertEqual(self.drv.locate(fcb(b"C", b"COM")), -1)
        self.assertEqual(self.drv.locate(fcb(b"a", b"com")), -1)   # регистр значим

    def test_name_truncated_to_fcb(self):
        # сравнение по 11 байтам FCB, как strncmp(...,11) в оригинале
        self.assertEqual(self.drv.locate(fcb(b"A", b"COM") + b"\xff\xff\xff"), 0)


class ReadFileTest(TempImageTestCase):
    """Последовательное чтение файла генератором 128-байтовых логических секторов."""

    def drive_with(self, rc, blocks):
        img = KdiImage().add(b"A", rc, blocks, ext=b"COM")
        for i, lba in enumerate(blocks):
            img.block(lba, bytes([0x10 + i]) * img.alsize)
        drv = self.drive(img.build())
        drv.read_dir()
        return drv

    def test_missing_file_returns_none(self):
        drv = self.drive_with(1, [1])
        self.assertIsNone(drv.read_file(fcb(b"ZZZ", b"COM")))

    def test_sector_stream(self):
        sectors = list(self.drive_with(20, [1, 2]).read_file(fcb(b"A", b"COM")))
        self.assertEqual(len(sectors), 20)
        self.assertTrue(all(len(s) == 128 for s in sectors))
        self.assertEqual(sectors[:16], [b"\x10" * 128] * 16)   # блок 1 целиком
        self.assertEqual(sectors[16:], [b"\x11" * 128] * 4)    # начало блока 2

    def test_stops_inside_block(self):
        # особенность оригинала: выдача кончается на records, хвост блока не уходит
        drv = self.drive_with(4, [1])
        self.assertEqual(list(drv.read_file(fcb(b"A", b"COM"))), [b"\x10" * 128] * 4)

    def test_blocks_follow_directory_order(self):
        drv = self.drive_with(20, [3, 1])           # не в порядке возрастания
        sectors = list(drv.read_file(fcb(b"A", b"COM")))
        self.assertEqual(sectors[:16], [b"\x10" * 128] * 16)   # сначала блок 3
        self.assertEqual(sectors[16:], [b"\x11" * 128] * 4)    # затем блок 1

    def test_records_more_than_blocks(self):
        # records больше, чем дают блоки: генератор останавливается на последнем блоке
        drv = self.drive_with(40, [1])
        self.assertEqual(len(list(drv.read_file(fcb(b"A", b"COM")))), 16)

    def test_missing_tail_padded_with_e5(self):
        # чего нет в образе — дополняется 0xE5 (в C читался мусор из буфера)
        img = KdiImage().add(b"A", 20, [1, 2], ext=b"COM")
        img.block(1, b"\xaa" * img.alsize)
        drv = self.drive(img.build(trim=img.alsize // 2))      # блок 1 оборван внутри
        drv.read_dir()
        sectors = list(drv.read_file(fcb(b"A", b"COM")))
        self.assertEqual(len(sectors), 20)
        self.assertEqual(sectors[:8], [b"\xaa" * 128] * 8)
        self.assertEqual(sectors[8:], [b"\xe5" * 128] * 12)

    def test_sequential_reads_of_two_files(self):
        img = (KdiImage().add(b"A", 2, [1], ext=b"COM")
                       .add(b"B", 3, [2], ext=b"COM"))
        img.block(1, b"\x11" * img.alsize)
        img.block(2, b"\x22" * img.alsize)
        drv = self.drive(img.build())
        drv.read_dir()
        self.assertEqual(list(drv.read_file(fcb(b"A", b"COM"))), [b"\x11" * 128] * 2)
        self.assertEqual(list(drv.read_file(fcb(b"B", b"COM"))), [b"\x22" * 128] * 3)


class BrokenImageTest(unittest.TestCase):
    """Битый образ — понятная ошибка вместо неопределённого поведения C."""

    def write(self, image):
        fd, path = tempfile.mkstemp(suffix=".kdi")
        with os.fdopen(fd, "wb") as f:
            f.write(image)
        self.addCleanup(_unlink, path)
        return path

    def test_header_shorter_than_info_sector(self):
        path = self.write(bytes(0x1F))              # инфосектор — 0x20 байт
        with self.assertRaises(ValueError) as ctx:
            KdiDrive(path)
        self.assertIn("слишком мал", str(ctx.exception))

    def test_zero_spt(self):
        with self.assertRaisesRegex(ValueError, "геометрия"):
            KdiDrive(self.write(info_sector(spt=0)))

    def test_zero_lspt(self):
        with self.assertRaisesRegex(ValueError, "геометрия"):
            KdiDrive(self.write(info_sector(lspt=0)))

    def test_zero_allocation_size(self):
        # bsh=8 даёт blocking=(1<<8)&0xFF=0 (как в C) и нулевую единицу распределения
        with self.assertRaisesRegex(ValueError, "единицы распределения"):
            KdiDrive(self.write(info_sector(bsh=8)))


class AllocMapTest(TempImageTestCase):
    """Карта блоков распределения и свободных входов каталога."""

    def test_max_block_from_dsm(self):
        drv = self.drive(KdiImage(tracks=40, dsm=10).build(full=True))
        self.assertEqual(drv.dsm, 10)
        self.assertEqual(drv.max_block(), 10)              # dsm дальше конца не пускает
        self.assertEqual(drv.free_blocks(), list(range(1, 11)))

    def test_max_block_from_image_size(self):
        # dsm=0 (не заполнен) - предел считается по фактическому концу образа
        drv = self.drive(KdiImage(tracks=40).build(full=True))
        self.assertEqual(drv.max_block(), (40 * 16 * 128) // 2048 - 3)   # 40 - 3 в начале
        self.assertEqual(drv.first_block(), 1)
        self.assertEqual(drv.free_blocks(), list(range(1, drv.max_block() + 1)))

    def test_first_block_skips_catalogue(self):
        # каталог из 128 входов занимает два блока по 2 КБ, а не один, как у
        # (drm+1)*16 в формуле - выделения должны начинаться за ним
        drv = self.drive(KdiImage(drm=127).build(full=True))
        dir_end = drv.dirpos + (drv.drm + 1) * 32
        self.assertEqual(drv.first_block(), 2)
        self.assertGreaterEqual(drv.block_address(drv.first_block()), dir_end)
        self.assertEqual(drv.free_blocks()[0], drv.first_block())

    def test_used_blocks_and_free_slots(self):
        img = (KdiImage(tracks=40).add(b"OLD", 5, [3, 4], ext=b"COM")
                                .add(b"NEW", 1, [], ext=b"COM"))
        drv = self.drive(img.build(full=True))
        name = fcb(b"OLD", b"COM")
        self.assertEqual(drv.used_blocks(), {3, 4})
        self.assertNotIn(3, drv.free_blocks())
        self.assertIn(3, drv.free_blocks(except_name=name))    # не считать этот файл
        self.assertEqual(drv.used_blocks(except_name=name), set())
        self.assertEqual(len(drv.free_slots()), 62)            # из 64 занято 2

    def test_block_address_matches_catalogue(self):
        img = KdiImage(tracks=40).add(b"OLD", 5, [7], ext=b"COM")
        img.block(7, b"\x5a" * img.alsize)
        drv = self.drive(img.build(full=True))
        self.assertEqual(self.raw(drv)[drv.block_address(7):][:128], b"\x5a" * 128)


class WriteTest(TempImageTestCase):
    """Запись файла в образ: блоки, экстенты, замена прежнего файла, ошибки места."""

    def writable(self, **kw):
        drv = self.drive(KdiImage(tracks=40, **kw).build(full=True))
        drv.read_dir()
        return drv

    def write_file(self, drv, base, data, ext=b"COM"):
        w = drv.open_write(fcb(base, ext))
        for i in range(0, len(data), 128):
            w.add(data[i:i + 128])
        return w

    def test_write_then_read_back(self):
        drv = self.writable()
        payload = bytes((i * 7) & 0xFF for i in range(16 * 128))   # ровно один блок
        res = self.write_file(drv, b"NEW", payload).commit()
        self.assertEqual(res["records"], 16)
        self.assertEqual(res["blocks"], [1])           # первый свободный блок
        self.assertEqual(b"".join(drv.read_file(fcb(b"NEW", b"COM"))), payload)
        off = drv.block_address(1)
        self.assertEqual(self.raw(drv)[off:off + len(payload)], payload)
        self.assertGreaterEqual(drv.locate(fcb(b"NEW", b"COM")), 0)

    def test_catalogue_entry_layout(self):
        drv = self.writable()
        self.write_file(drv, b"NEW", bytes(20 * 128)).commit()
        e = drv.entry_at(drv.entry_positions(fcb(b"NEW", b"COM"))[0])
        self.assertEqual(e[0], 0)                                   # user-область
        self.assertEqual(e[1:12], fcb(b"NEW", b"COM"))
        self.assertEqual(e[0x0C], 0)                                # номер экстента
        self.assertEqual(e[0x0F], 20)                               # записей в экстенте
        self.assertEqual(struct.unpack_from("<8H", e, 0x10), (1, 2, 0, 0, 0, 0, 0, 0))

    def test_multiple_extents(self):
        drv = self.writable()
        payload = bytes((i * 3) & 0xFF for i in range(300 * 128))
        res = self.write_file(drv, b"BIG", payload).commit()
        self.assertEqual(res["records"], 300)
        self.assertEqual(res["blocks"], list(range(1, 20)))         # ceil(300/16)
        positions = drv.entry_positions(fcb(b"BIG", b"COM"))
        self.assertEqual(len(positions), 3)                         # 128+128+44
        heads = [drv.entry_at(n) for n in positions]
        self.assertEqual([e[0x0C] for e in heads], [0, 1, 2])       # номера экстентов
        self.assertEqual([e[0x0F] for e in heads], [128, 128, 44])
        sectors = list(drv.read_file(fcb(b"BIG", b"COM")))
        self.assertEqual(b"".join(sectors), payload[:300 * 128])

    def test_replaces_existing_file(self):
        img = KdiImage(tracks=40).add(b"OLD", 32, [5, 6], ext=b"COM")
        img.block(5, b"\xa5" * img.alsize)
        img.block(6, b"\x5a" * img.alsize)
        drv = self.drive(img.build(full=True))
        drv.read_dir()
        old = fcb(b"OLD", b"COM")
        self.write_file(drv, b"OLD", bytes(16 * 128)).commit()
        self.assertEqual(len(drv.files), 1)                         # не дубликат
        self.assertEqual(drv.files[0]["records"], 16)
        self.assertEqual(drv.files[0]["blocks"], [1])               # прежние освобождены
        self.assertEqual(drv.used_blocks(), {1})
        self.assertIn(5, drv.free_blocks())
        self.assertEqual(b"".join(drv.read_file(old)), bytes(16 * 128))

    def test_empty_file(self):
        drv = self.writable()
        res = drv.open_write(fcb(b"E", b"COM")).commit()
        self.assertEqual(res["records"], 0)
        self.assertEqual(res["blocks"], [])
        self.assertEqual(list(drv.read_file(fcb(b"E", b"COM"))), [])
        self.assertGreaterEqual(drv.locate(fcb(b"E", b"COM")), 0)

    def test_short_record_is_padded(self):
        drv = self.writable()
        w = drv.open_write(fcb(b"S", b"COM"))
        w.add(b"abc")
        w.commit()
        self.assertEqual(list(drv.read_file(fcb(b"S", b"COM"))),
                         [b"abc".ljust(128, b"\x00")])

    def test_without_commit_image_unchanged(self):
        drv = self.writable()
        before = self.raw(drv)
        w = self.write_file(drv, b"NEW", bytes(10 * 128))
        del w                                          # клиент не прислал закрытие
        self.assertEqual(self.raw(drv), before)
        self.assertEqual(drv.files, [])

    def test_disk_full_changes_nothing(self):
        drv = self.drive(KdiImage(tracks=4).build(full=True))   # 1 свободный блок
        drv.read_dir()
        before = self.raw(drv)
        w = self.write_file(drv, b"BIG", bytes(40 * 128))        # нужно 3 блока
        with self.assertRaisesRegex(KdiError, "не хватает места на образе"):
            w.commit()
        self.assertEqual(self.raw(drv), before)

    def test_directory_full_changes_nothing(self):
        img = KdiImage(tracks=40, drm=7)                          # 8 входов каталога
        for i, base in enumerate([b"F1", b"F2", b"F3", b"F4", b"F5", b"F6", b"F7"]):
            img.add(base, 1, [], ext=b"COM")
        drv = self.drive(img.build(full=True))
        drv.read_dir()
        before = self.raw(drv)
        w = self.write_file(drv, b"BIG", bytes(300 * 128))        # нужно 3 входа
        with self.assertRaisesRegex(KdiError, "не хватает места в каталоге"):
            w.commit()
        self.assertEqual(self.raw(drv), before)

    def test_bad_names_rejected(self):
        drv = self.writable()
        for bad in (b"           ", b"\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01\x01",
                    b""):
            with self.subTest(name=bad):
                with self.assertRaises(KdiError):
                    drv.open_write(bad)

    def test_name_padded_to_fcb(self):
        drv = self.writable()
        w = drv.open_write(b"A")                       # короткое имя - как из FCB
        self.assertEqual(w.name, fcb(b"A"))
        w.commit()
        self.assertGreaterEqual(drv.locate(fcb(b"A")), 0)


class DeleteRenameTest(TempImageTestCase):
    """Удаление и переименование файлов в каталоге образа."""

    def setUp(self):
        super().setUp()
        img = KdiImage(tracks=40)
        img.add(b"A", 20, [1, 2], ext=b"COM", ex=0)
        img.add(b"B", 128, list(range(3, 11)), ext=b"DOC", ex=0)
        img.add(b"B", 2, [11], ext=b"DOC", ex=1)
        self.drv = self.drive(img.build(full=True))
        self.drv.read_dir()

    def test_delete_marks_all_used(self):
        name = fcb(b"B", b"DOC")
        self.assertEqual(self.drv.delete(name), 2)               # оба экстента
        self.assertEqual(self.drv.entry_positions(name), [])
        self.assertEqual([f["name"] for f in self.drv.files], [fcb(b"A", b"COM")])
        self.assertEqual(self.drv.used_blocks(), {1, 2})         # блоки освободились
        raw = self.raw(self.drv)
        for n in (1, 2):                                         # входы каталога
            self.assertEqual(raw[self.drv.dirpos + 32 * n], 0xE5)

    def test_delete_missing(self):
        with self.assertRaisesRegex(KdiError, "не найден"):
            self.drv.delete(fcb(b"NOPE", b"COM"))

    def test_rename_all_extents(self):
        self.assertEqual(self.drv.rename(fcb(b"B", b"DOC"), b"ZZZZZZZ DOC"), 2)
        new = fcb(b"ZZZZZZZ", b"DOC")
        self.assertEqual(self.drv.entry_positions(new), [1, 2])  # оба входа переименованы
        self.assertEqual(self.drv.files[1]["records"], 130)      # склейка осталась
        self.assertEqual(self.drv.files[1]["blocks"], list(range(3, 12)))
        self.assertEqual(self.drv.entry_positions(fcb(b"B", b"DOC")), [])

    def test_rename_collision(self):
        with self.assertRaisesRegex(KdiError, "уже есть"):
            self.drv.rename(fcb(b"A", b"COM"), fcb(b"B", b"DOC"))

    def test_rename_missing(self):
        with self.assertRaisesRegex(KdiError, "не найден"):
            self.drv.rename(fcb(b"NOPE", b"COM"), fcb(b"OK", b"COM"))

    def test_rename_bad_new_name(self):
        with self.assertRaises(KdiError):
            self.drv.rename(fcb(b"A", b"COM"), b"           ")


# ---------------------------------------------------------------------------
# Демо-диск disk/sys.kdi: 5 физических секторов по 1024 байта, 80 дорожек,
# 2 стороны, блоки по 2 КБ, каталог 128 входов. Значения расшифрованы из образа.
# ---------------------------------------------------------------------------

SAMPLE_GEOMETRY = {
    "count": 10, "secsz": 1024, "sides": 1, "spt": 5, "tracks": 80, "lspt": 40,
    "bsh": 4, "drm": 127, "systracks": 2, "blocking": 16, "alsize": 2048,
    "reserved": 6, "dirblocks": 1,
}
SAMPLE_DIRPOS = 0x2800          # (reserved - dirblocks) * alsize
SAMPLE_NFILES = 35
# КС всех байт файла (& 0xFF) — то, что монитор дописывает после пересылки.
SAMPLE_CSUM = {b"DED     COM": 80, b"PIP     COM": 217, b"DUMP    COM": 26}


def layout(raw):
    """Раскладка образа по docs/KDI.md — независимая реализация для сверки."""
    secsz = SECTOR_SIZES[raw[0x0A] & 3]
    count = struct.unpack_from("<H", raw, 0x04)[0]
    spt = struct.unpack_from("<H", raw, 0x0C)[0]
    lspt = struct.unpack_from("<H", raw, 0x10)[0]
    bsh = raw[0x12]
    drm = struct.unpack_from("<H", raw, 0x17)[0]
    alsize = ((1 << bsh) & 0xFF) * (spt * secsz // lspt)
    return {"secsz": secsz, "spt": spt, "drm": drm, "alsize": alsize,
            "reserved": (count * secsz + (drm + 1) * 16) // alsize,
            "dirblocks": (drm + 1) * 16 // alsize,
            # начало каталога = CHS-граница после системных дорожек
            "dirpos": struct.unpack_from("<H", raw, 0x1D)[0] * spt * secsz}


def raw_dir(raw, lay):
    """Каталог образа: ({имя: [число записей, [блоки]]}, порядок имён)."""
    files, order = {}, []
    for n in range(lay["drm"] + 1):
        e = raw[lay["dirpos"] + 32 * n: lay["dirpos"] + 32 * n + 32]
        if len(e) < 32 or e[0] == 0xE5:
            continue
        name = bytes(b & 0x7F for b in e[1:12])
        rc = e[0x0F]
        blocks = [b for b in struct.unpack_from("<8H", e, 0x10) if b]
        if name in files:
            files[name][0] += rc
            files[name][1] += blocks
        else:
            files[name] = [rc, list(blocks)]
            order.append(name)
    return files, order


def raw_file(raw, lay, records, blocks):
    """Файл как склейка блоков данных (с дополнением 0xE5 вне образа)."""
    data = bytearray()
    for lba in blocks:
        off = (lay["reserved"] + lba - lay["dirblocks"]) * lay["alsize"]
        chunk = raw[off:off + lay["alsize"]]
        data += chunk + b"\xe5" * (lay["alsize"] - len(chunk))
    return bytes(data[:records * 128])


@unittest.skipUnless(os.path.exists(SAMPLE), "нет демо-диска disk/sys.kdi")
class SampleDiskTest(unittest.TestCase):
    """Настоящий образ: геометрия, каталог со склейкой экстентов, чтение файлов."""

    @classmethod
    def setUpClass(cls):
        with open(SAMPLE, "rb") as f:
            cls.raw = f.read()
        cls.lay = layout(cls.raw)
        cls.want, cls.order = raw_dir(cls.raw, cls.lay)
        cls.drv = KdiDrive(SAMPLE)
        cls.drv.read_dir()

    @classmethod
    def tearDownClass(cls):
        cls.drv.close()

    def block_off(self, lba):
        return (self.drv.reserved + lba - self.drv.dirblocks) * self.drv.alsize

    def test_geometry(self):
        for attr, want in SAMPLE_GEOMETRY.items():
            with self.subTest(attr=attr):
                self.assertEqual(getattr(self.drv, attr), want)

    def test_image_size_matches_chs(self):
        g = SAMPLE_GEOMETRY
        self.assertEqual(g["spt"] * (g["sides"] + 1) * g["tracks"] * g["secsz"],
                         len(self.raw))

    def test_catalogue_position_by_chs(self):
        # каталог стоит ровно на CHS-границе после 2 системных дорожек:
        # systracks=2 при двух сторонах -> cyl=1, head=0, сектор 1
        self.assertEqual(self.lay["dirpos"], SAMPLE_DIRPOS)
        self.drv.seek_chs(1, 0, 1)
        self.assertEqual(self.drv.file.tell(), SAMPLE_DIRPOS)
        entry = self.drv.file.read(12)
        self.assertEqual(entry[0], 0)                      # номер user-раздела
        self.assertEqual(entry[1:12], b"DED     COM")

    def test_catalogue(self):
        names = [f["name"] for f in self.drv.files]
        self.assertEqual(len(names), SAMPLE_NFILES)
        self.assertEqual(names, self.order)
        self.assertEqual(len(set(names)), len(names))
        for name in names:
            with self.subTest(name=name):
                self.assertRegex(name, rb"^[A-Z0-9 ]{11}$")

    def test_known_files_and_extents(self):
        by = {f["name"]: f for f in self.drv.files}
        for known in (b"DED     COM", b"PIP     COM", b"DDT     COM",
                      b"BIOS    DOC", b"FORMAT  COM"):
            self.assertIn(known, by)
        self.assertEqual(by[b"DED     COM"]["records"], 18)      # 16 + 2, два блока
        self.assertEqual(by[b"DED     COM"]["blocks"], [2, 3])
        self.assertEqual(by[b"DUMP    COM"]["records"], 4)       # меньше блока
        self.assertEqual(by[b"BIOS    DOC"]["records"], 558)     # склейка 5 экстентов
        self.assertEqual(by[b"BIOS    DOC"]["blocks"], list(range(4, 39)))

    def test_reads_match_raw_image(self):
        for f in self.drv.files:
            with self.subTest(name=f["name"].decode("ascii", "replace")):
                recs, blocks = self.want[f["name"]]
                self.assertEqual(b"".join(self.drv.read_file(f["name"])),
                                 raw_file(self.raw, self.lay, recs, blocks))

    def test_sector_count_and_size(self):
        for f in self.drv.files:
            with self.subTest(name=f["name"].decode("ascii", "replace")):
                sectors = list(self.drv.read_file(f["name"]))
                self.assertEqual(len(sectors), f["records"])
                self.assertTrue(all(len(s) == 128 for s in sectors))

    def test_stop_inside_block(self):
        # DUMP COM (4 сектора) и DED COM (18) — выдача обрывается в середине блока
        dump = self.block_off(91)
        self.assertEqual(b"".join(self.drv.read_file(b"DUMP    COM")),
                         self.raw[dump:dump + 4 * 128])
        ded = b"".join(self.drv.read_file(b"DED     COM"))
        self.assertEqual(len(ded), 18 * 128)
        self.assertEqual(ded[:16 * 128], self.raw[self.block_off(2):][:16 * 128])
        self.assertEqual(ded[16 * 128:], self.raw[self.block_off(3):][:2 * 128])

    def test_transfer_checksums(self):
        for name, want in SAMPLE_CSUM.items():
            with self.subTest(name=name.decode("ascii")):
                self.assertEqual(sum(b"".join(self.drv.read_file(name))) & 0xFF, want)

    def test_program_entry_point(self):
        # PIP COM начинается с JMP 04CE — верный признак правильного чтения байтов
        self.assertEqual(next(self.drv.read_file(b"PIP     COM"))[:3], b"\xc3\xce\x04")

    def test_blocks_inside_image(self):
        for f in self.drv.files:
            for lba in f["blocks"]:
                with self.subTest(name=f["name"].decode("ascii"), lba=lba):
                    self.assertLessEqual(self.block_off(lba) + self.drv.alsize,
                                         len(self.raw))

    def test_locate_on_sample(self):
        self.assertEqual(self.drv.locate(b"PIP     COM"),
                         self.order.index(b"PIP     COM"))
        self.assertEqual(self.drv.locate(fcb(b"NOPE", b"COM")), -1)

    def test_image_untouched_after_reads(self):
        # образ открыт на чтение с записью (как в оригинале) — содержимое не меняется
        for f in self.drv.files:
            list(self.drv.read_file(f["name"]))
        with open(SAMPLE, "rb") as f:
            self.assertEqual(f.read(), self.raw)


@unittest.skipUnless(os.path.exists(SAMPLE), "нет демо-диска disk/sys.kdi")
class SampleCopyWriteTest(unittest.TestCase):
    """Запись в копию демо-диска: файл появился, остальной диск не тронут."""

    NAME = fcb(b"MKDI", b"DOC")

    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix=".kdi")
        os.close(fd)
        shutil.copyfile(SAMPLE, self.path)
        with open(SAMPLE, "rb") as f:
            self.original = f.read()
        self.drv = KdiDrive(self.path)
        self.drv.read_dir()
        # снимок всего диска до записи — им и сверяем после
        self.before = {f["name"]: b"".join(self.drv.read_file(f["name"]))
                       for f in self.drv.files}

    def tearDown(self):
        self.drv.close()
        _unlink(self.path)

    def write_test_file(self, records=200):
        payload = bytes((i * 31 + 7) & 0xFF for i in range(records * 128))
        w = self.drv.open_write(self.NAME)
        for i in range(records):
            w.add(payload[i * 128:(i + 1) * 128])
        return w, payload

    def test_write_on_real_geometry(self):
        self.assertEqual(self.drv.first_block(), 2)   # каталог занимает два блока
        want = self.drv.free_blocks()[0]
        w, payload = self.write_test_file()
        res = w.commit()
        self.assertEqual(res["records"], 200)
        self.assertEqual(len(res["blocks"]), 13)      # ceil(200 / 16) блоков по 2 КБ
        self.assertEqual(res["blocks"][0], want)      # отдаётся первый свободный блок
        dir_end = self.drv.dirpos + (self.drv.drm + 1) * 32
        self.assertGreaterEqual(self.drv.block_address(min(res["blocks"])), dir_end)
        fresh = KdiDrive(self.path)                   # перечитать образ с диска
        self.addCleanup(fresh.close)
        fresh.read_dir()
        self.assertEqual(fresh.files[fresh.locate(self.NAME)]["records"], 200)
        self.assertEqual(b"".join(fresh.read_file(self.NAME)), payload)

    def test_other_files_survive_write(self):
        w, _ = self.write_test_file()
        w.commit()
        fresh = KdiDrive(self.path)
        self.addCleanup(fresh.close)
        fresh.read_dir()
        self.assertEqual(len(fresh.files), len(self.before) + 1)
        for name, data in self.before.items():
            with self.subTest(name=name.decode("ascii")):
                self.assertIsNotNone(fresh.read_file(name))
                self.assertEqual(b"".join(fresh.read_file(name)), data)

    def test_delete_frees_blocks_on_real_disk(self):
        blocks = set(self.drv.files[self.drv.locate(b"STAT    COM")]["blocks"])
        self.assertTrue(blocks)
        self.assertEqual(self.drv.delete(b"STAT    COM"), 1)
        self.assertNotIn(b"STAT    COM", [f["name"] for f in self.drv.files])
        free = self.drv.free_blocks()
        for lba in blocks:
            self.assertIn(lba, free)

    def test_rename_on_real_disk(self):
        data = b"".join(self.drv.read_file(b"STAT    COM"))
        self.assertEqual(self.drv.rename(b"STAT    COM", fcb(b"STAT2", b"COM")), 1)
        self.assertEqual(self.drv.locate(b"STAT    COM"), -1)
        self.assertEqual(b"".join(self.drv.read_file(fcb(b"STAT2", b"COM"))), data)
        with self.assertRaisesRegex(KdiError, "уже есть"):
            self.drv.rename(fcb(b"PIP", b"COM"), fcb(b"STAT2", b"COM"))

    def test_catalogue_stays_clean(self):
        w, _ = self.write_test_file()
        w.commit()
        self.assertEqual(len(self.drv.files), len(self.before) + 1)
        for f in self.drv.files:
            with self.subTest(name=f["name"]):
                self.assertRegex(f["name"], rb"^[A-Z0-9 ]{11}$")

    def test_original_sample_not_modified(self):
        w, _ = self.write_test_file()
        w.commit()
        self.drv.delete(b"STAT    COM")
        with open(SAMPLE, "rb") as f:
            self.assertEqual(f.read(), self.original)


if __name__ == "__main__":
    unittest.main(verbosity=2)
