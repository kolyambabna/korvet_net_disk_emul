# -*- coding: utf-8 -*-
"""Неблокирующий ввод консольных клавиш команд.

Используется главным циклом монитора для клавиш 'r' (перезагрузка ОС)
и 'x' (выход). На Linux/macOS терминал переводится в cbreak-режим,
на Windows используется msvcrt. Если стандартный ввод не является
терминалом, клавиши просто недоступны.
"""

import select
import sys


class ConsoleKeys:
    """Неблокиющее чтение одиночных нажатий клавиш консоли."""

    def __init__(self):
        self._enabled = False
        self._old = None
        self._termios = None
        if sys.platform == "win32":
            import msvcrt  # консольный буфер Windows читается без смены режима
            self._enabled = True
            self._read_key = lambda: msvcrt.getwch() if msvcrt.kbhit() else None
            return
        try:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._termios = termios
            self._enabled = True
        except Exception:  # stdin не терминал - работаем без командных клавиш
            self._old = None
        if self._enabled:
            def read_key():
                if select.select([sys.stdin], [], [], 0)[0]:
                    return sys.stdin.read(1)
                return None
            self._read_key = read_key
        else:
            self._read_key = lambda: None

    def key(self):
        """Вернуть нажатую клавишу или None. Ctrl+C -> KeyboardInterrupt."""
        if not self._enabled:
            return None
        ch = self._read_key()
        if ch == "\x03":  # Ctrl+C
            raise KeyboardInterrupt
        return ch

    def close(self):
        if self._old is not None and self._termios is not None:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)
            self._old = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
