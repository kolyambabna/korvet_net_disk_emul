# -*- coding: utf-8 -*-
"""Общие утилиты монитора: отладочная печать и аварийный выход.

Аналог одноименных макросов dprintf/abort из pcndr.h.
"""

import sys

serial_timeout = 1  # таймаут ожидания очередного байта из сети в 0.1 с.

debug = 0  # флаг разрешения отладочной печати (устанавливается ключом -d)


def dprintf(fmt, *args):
    """Отладочная печать (при установленном флаге debug)."""
    if debug:
        print(fmt % args if args else fmt, end="", flush=True)


def abort(fmt, *args):
    """Аварийный выход: печать сообщения и завершение с кодом 1."""
    print(fmt % args if args else fmt, end="", flush=True)
    sys.exit(1)
