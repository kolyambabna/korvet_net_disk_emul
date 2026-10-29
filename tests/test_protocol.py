# -*- coding: utf-8 -*-
"""Тесты протокола ОПТС 2.0 (lan.py) и читателя образов KDI (kdi.py).

pyserial не требуется: используется эмуляция последовательного порта.
Запуск из корня проекта:  python -m unittest discover -s py/tests -v
"""

import importlib
import os
import struct
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from kdi import KdiDrive  # noqa: E402
from lan import Lan       # noqa: E402


class FakeSerial:
    """Эмуляция порта pyserial: таймаут чтения заменяется концом очереди rx."""

    def __init__(self, rx=b""):
        self.tx = bytearray()
        self._rx = list(rx)
        self.timeout = None

    def read(self, size=1):
        out = bytes(self._rx[:size])
        del self._rx[:size]
        return out

    def write(self, data):
        self.tx += data
        return len(data)

    def flush(self):
        pass


def wire_packet(typ):
    """Собрать «на проводе» корректный 6-байтовый пакет типа typ."""
    lan = Lan(FakeSerial(), 2)
    lan.pkt_y(ord(typ))
    return bytes(lan.ser.tx)


class ChecksumTest(unittest.TestCase):
    def test_known_vector(self):
        # sum(0x41+0x42)=0x83; (0x83>>6)+0x83=0x85; 0x85&0x3F=5; +0x20=0x25
        self.assertEqual(Lan.checksum([0, 0x41, 0x42, 0], 3), 0x25)

    def test_zero_sum(self):
        self.assertEqual(Lan.checksum([0, 0, 0, 0], 3), 0x20)


class ReadpktTest(unittest.TestCase):
    def test_valid_packet(self):
        lan = Lan(FakeSerial(wire_packet("Y")), 2)
        self.assertEqual(lan.readpkt(), 6)
        self.assertEqual(lan.ibuf[3], ord("Y"))

    def test_garbage_before_sync(self):
        lan = Lan(FakeSerial(b"\x05\xff" + wire_packet("Y")), 2)
        self.assertEqual(lan.readpkt(), 6)

    def test_timeout(self):
        lan = Lan(FakeSerial(b""), 2)
        self.assertEqual(lan.readpkt(), 0)

    def test_bad_checksum(self):
        pkt = bytearray(wire_packet("Y"))
        pkt[4] ^= 0xFF
        lan = Lan(FakeSerial(bytes(pkt)), 2)
        self.assertEqual(lan.readpkt(), -1)


class SendpktTest(unittest.TestCase):
    def test_packet_layout(self):
        lan = Lan(FakeSerial(), 2)
        lan.pkt_y(0x21)
        tx = lan.ser.tx
        self.assertEqual(tx[0], 0x01)                 # синхробайт
        self.assertEqual(tx[1], 6 + 0x1D)             # длина
        self.assertEqual(tx[2], 0x21)                 # номер пакета
        self.assertEqual(tx[3], ord("Y"))
        self.assertEqual(tx[5], 0x0D)                 # конец пакета
        self.assertEqual(tx[4], Lan.checksum(tx, 4))  # КС


class PktDTest(unittest.TestCase):
    def test_plain_and_hash_escape(self):
        lan = Lan(FakeSerial(), 2)
        consumed = lan.pkt_d(0x20, b"A#B", 3)
        self.assertEqual(consumed, 3)
        self.assertEqual(lan.ser.tx[4:8], b"A##B")  # '#' удваивается

    def test_ctl_and_high_bit(self):
        lan = Lan(FakeSerial(), 2)
        consumed = lan.pkt_d(0x20, b"\x01\x8a", 2)
        self.assertEqual(consumed, 2)
        # 0x01 -> '#'+('A'); 0x8a -> '#' + (0x0a+0x40|0x80)=0xCA
        self.assertEqual(lan.ser.tx[4:8], b"#A#\xca")

    def test_maxlen(self):
        lan = Lan(FakeSerial(), 2)
        consumed = lan.pkt_d(0x20, b"A" * 100, 100)
        self.assertEqual(consumed, 84)  # 4 байта заголовка + 84 данных = 0x58

    def test_send_data_with_ack(self):
        lan = Lan(FakeSerial(wire_packet("Y") * 5), 2)
        self.assertEqual(lan.send_data(b"A" * 300, 300), 300)


def dentry(user, name, ex, rc, blocks):
    """32-байтовый описатель файла в каталоге CP/M."""
    e = bytearray(32)
    e[0] = user
    e[1:12] = name
    e[0x0C] = ex
    e[0x0F] = rc
    e[0x10:0x20] = struct.pack("<8H", *(list(blocks) + [0] * (8 - len(blocks))))
    return e


class KdiTest(unittest.TestCase):
    """Синтетический KDI: 128-байтные сектора, 16 с/д, блок 2048, каталог 64 входа."""

    def setUp(self):
        sec1 = bytearray(128)
        sec1[0x00:0x02] = struct.pack("<H", 0x0100)  # LoadAdress
        sec1[0x02:0x04] = struct.pack("<H", 0x0100)  # RunAdress
        sec1[0x04:0x06] = struct.pack("<H", 26)      # Count
        sec1[0x0A] = 0                               # физический сектор 128 байт
        sec1[0x0B] = 0                               # односторонний
        sec1[0x0C:0x0E] = struct.pack("<H", 16)      # spt
        sec1[0x0E:0x10] = struct.pack("<H", 40)      # tracks
        sec1[0x10:0x12] = struct.pack("<H", 16)      # lspt
        sec1[0x12] = 4                               # bsh: блок = 16 секторов = 2048 байт
        sec1[0x17:0x19] = struct.pack("<H", 63)      # drm
        sec1[0x1D:0x1F] = struct.pack("<H", 2)       # systracks

        img = bytearray(sec1)
        img += bytes(128 * (16 * 2 - 1))  # остаток двух системных дорожек
        # каталог: FOO COM из двух экстентов (блоки 1 и 2), далее свободные входы
        img += dentry(0, b"FOO     COM", 0, 16, [1])
        img += dentry(0, b"FOO     COM", 1, 8, [2])
        for _ in range(64 - 2):
            img += dentry(0xE5, b"           ", 0, 0, [])
        # area данных: reserved=2 блока по 2048, блок #1 сразу за каталогом
        img += b"\x11" * 2048  # блок 1
        img += b"\x22" * 2048  # блок 2

        fd, self.path = tempfile.mkstemp(suffix=".kdi")
        with os.fdopen(fd, "wb") as f:
            f.write(img)
        self.drv = KdiDrive(self.path)
        self.drv.read_dir()

    def tearDown(self):
        self.drv.close()
        os.unlink(self.path)

    def test_params(self):
        self.assertEqual(self.drv.blocking, 16)
        self.assertEqual(self.drv.alsize, 2048)
        self.assertEqual(self.drv.reserved, 2)
        self.assertEqual(self.drv.dirblocks, 0)

    def test_directory_parsed(self):
        self.assertEqual(len(self.drv.files), 1)
        f = self.drv.files[0]
        self.assertEqual(f["name"], b"FOO     COM")
        self.assertEqual(f["records"], 24)  # 16 + 8 секторов из двух экстентов
        self.assertEqual(f["blocks"], [1, 2])

    def test_read_file(self):
        sectors = list(self.drv.read_file(b"FOO     COM"))
        self.assertEqual(len(sectors), 24)
        self.assertTrue(all(s == b"\x11" * 128 for s in sectors[:16]))
        self.assertTrue(all(s == b"\x22" * 128 for s in sectors[16:]))

    def test_file_not_found(self):
        self.assertIsNone(self.drv.read_file(b"BAR     COM"))


class AckTest(unittest.TestCase):
    """Подтверждение монитора: сколько байтов уходит в линию и как (pcndr.send_ack).

    Одиночный 'Y' залипает в буфере драйвера USB-RS232 (latency timer), поэтому
    подтверждение отправляется пачкой и с flush; по умолчанию — один байт, как в C.
    """

    @classmethod
    def setUpClass(cls):
        if "serial" not in sys.modules:  # pyserial в тестах не нужен
            stub = types.ModuleType("serial")
            stub.EIGHTBITS = 8
            stub.STOPBITS_ONE = 1
            stub.PARITY_ODD = "O"
            stub.SerialException = IOError
            stub.Serial = lambda **kw: None
            sys.modules["serial"] = stub
        cls.pcndr = importlib.import_module("pcndr")

    def parse(self, *argv):
        saved, sys.argv = sys.argv, ["pcndr.py"] + list(argv)
        try:
            return self.pcndr.parse_args()
        finally:
            sys.argv = saved

    def test_defaults_match_c_version(self):
        args = self.parse("-a", "disk/sys.kdi", "-s", "COM5")
        self.assertEqual(args.yy, 1)
        self.assertEqual(args.posty, 2.0)

    def test_tuning_keys(self):
        args = self.parse("-s", "COM5", "-n", "2", "-yy", "2", "-posty", "20")
        self.assertEqual((args.yy, args.posty), (2, 20.0))

    def test_ack_is_repeated_for_both_answers(self):
        for ch, count, wire in ((b"Y", 1, b"Y"), (b"Y", 2, b"YY"),
                                (b"N", 2, b"NN"), (b"N", 3, b"NNN")):
            port = FlushCountingSerial()
            self.pcndr.send_ack(port, ch, count)
            self.assertEqual(bytes(port.tx), wire)
            self.assertEqual(port.flushes, 1)  # без flush байт остаётся в буфере


class FlushCountingSerial(FakeSerial):
    def __init__(self):
        super().__init__()
        self.flushes = 0

    def flush(self):
        self.flushes += 1


if __name__ == "__main__":
    unittest.main()
