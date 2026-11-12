#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Виртуальные Modbus-приборы — учебный симулятор шины.

Запуск:
    python modbus_sim.py --port COM3              # Modbus RTU на COM-порту
    python modbus_sim.py --port COM3 --baud 19200
    python modbus_sim.py --tcp 502                # Modbus TCP (без железа)

После запуска откройте в браузере:  http://127.0.0.1:8080

Приборы описываются в файле devices.json — добавляйте свои без программирования.
Зависимости: стандартная библиотека Python; для COM-порта нужен pyserial
( pip install pyserial ). Режим --tcp работает вообще без установки чего-либо.
"""

import argparse
import json
import os
import select
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Хранилище приборов
# ---------------------------------------------------------------------------

# Наши названия типов в JSON -> внутренние короткие коды
KINDS = {"coil": "co", "discrete": "di", "holding": "hr", "input": "ir"}
KIND_NAMES = {"co": "coil", "di": "discrete input",
              "hr": "holding register", "ir": "input register"}
# Функции чтения Modbus -> тип данных
KIND_OF_FC = {0x01: "co", 0x02: "di", 0x03: "hr", 0x04: "ir"}

LOCK = threading.Lock()          # один замок на все регистры — для учебного
                                  # инструмента этого более чем достаточно

# Состояние шины для веб-панели (зелёная/красная плашка)
BUS_STATUS = {"ok": False, "text": "шина не запущена"}


def set_bus(ok, text):
    BUS_STATUS["ok"] = ok
    BUS_STATUS["text"] = text


class Device:
    """Один виртуальный прибор на шине (один Modbus-адрес)."""

    def __init__(self, cfg):
        self.addr = cfg["addr"]
        self.name = cfg.get("name", "Прибор %d" % self.addr)
        self.image = cfg.get("image", "")
        self.regs = cfg.get("regs", [])

        # Размер каждой области = максимальный номер регистра + 1
        size = {"co": 0, "di": 0, "hr": 0, "ir": 0}
        for r in self.regs:
            k = KINDS[r["type"]]
            size[k] = max(size[k], r["num"] + 1)
        self.values = {k: [0] * n for k, n in size.items()}

        # Начальные значения (init задаётся в «человеческих» единицах)
        with LOCK:
            for r in self.regs:
                if "init" in r:
                    self.set_human(r, r["init"])

    # --- пересчёт «человеческие единицы» <-> «сырые» значения Modbus ---
    @staticmethod
    def raw_of(reg, human):
        if KINDS[reg["type"]] in ("co", "di"):
            return 1 if human else 0
        return max(0, min(65535, round(human * reg.get("scale", 1))))

    @staticmethod
    def human_of(reg, raw):
        if KINDS[reg["type"]] in ("co", "di"):
            return bool(raw)
        return raw / reg.get("scale", 1)

    def set_human(self, reg, human):
        self.values[KINDS[reg["type"]]][reg["num"]] = self.raw_of(reg, human)

    def set_bit(self, reg, bit, value):
        """Установить/сбросить один бит регистра (для виджета bitfield)."""
        if bit < 0 or bit >= len(reg.get("bits", [])):
            raise ValueError("нет бита %d" % bit)
        k = KINDS[reg["type"]]
        cur = self.values[k][reg["num"]]
        cur = (cur | (1 << bit)) if value else (cur & ~(1 << bit))
        self.values[k][reg["num"]] = cur & 0xFFFF

    def read(self, kind, start, qty):
        vals = self.values[kind]
        if start + qty > len(vals):
            raise ModbusError(2)          # illegal data address
        return vals[start:start + qty]

    def write(self, kind, start, new_values):
        vals = self.values[kind]
        if start + len(new_values) > len(vals):
            raise ModbusError(2)
        vals[start:start + len(new_values)] = new_values


class ModbusError(Exception):
    def __init__(self, code):
        super().__init__("Modbus exception %d" % code)
        self.code = code


def load_devices(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    devices = {}
    for d in cfg["devices"]:
        dev = Device(d)
        if dev.addr in devices:
            sys.exit("Ошибка: адрес %d встречается дважды" % dev.addr)
        devices[dev.addr] = dev
    return devices


# ---------------------------------------------------------------------------
# Ядро протокола: разбор PDU (одинаково для RTU и TCP)
# ---------------------------------------------------------------------------

def crc16(data):
    """CRC16 Modbus: полином 0xA001, начальное значение 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def pack_bits(bits):
    out = bytearray((len(bits) + 7) // 8)
    for i, b in enumerate(bits):
        if b:
            out[i // 8] |= 1 << (i % 8)
    return bytes(out)


def unpack_bits(data, qty):
    return [(data[i // 8] >> (i % 8)) & 1 for i in range(qty)]


def handle_read(dev, fc, body):
    """Функции 01-04: чтение co/di/hr/ir."""
    if len(body) != 4:
        raise ModbusError(3)
    start, qty = int.from_bytes(body[0:2], "big"), int.from_bytes(body[2:4], "big")
    kind = KIND_OF_FC[fc]
    if qty < 1 or qty > (2000 if kind in ("co", "di") else 125):
        raise ModbusError(3)
    with LOCK:
        vals = dev.read(kind, start, qty)
    if kind in ("co", "di"):
        packed = pack_bits(vals)
    else:
        packed = b"".join(v.to_bytes(2, "big") for v in vals)
    return bytes([fc, len(packed)]) + packed


def handle_write(dev, fc, body):
    """Функции 05, 06, 15(0x0F), 16(0x10): запись co/hr."""
    if fc == 0x05:                          # write single coil
        if len(body) != 4:
            raise ModbusError(3)
        num, raw = int.from_bytes(body[0:2], "big"), int.from_bytes(body[2:4], "big")
        if raw not in (0x0000, 0xFF00):
            raise ModbusError(3)
        with LOCK:
            dev.write("co", num, [1 if raw else 0])
        return bytes([fc]) + body           # ответ = эхо запроса

    if fc == 0x06:                          # write single register
        if len(body) != 4:
            raise ModbusError(3)
        num, val = int.from_bytes(body[0:2], "big"), int.from_bytes(body[2:4], "big")
        with LOCK:
            dev.write("hr", num, [val])
        return bytes([fc]) + body

    if fc == 0x0F:                          # write multiple coils
        if len(body) < 5:
            raise ModbusError(3)
        start = int.from_bytes(body[0:2], "big")
        qty = int.from_bytes(body[2:4], "big")
        if body[4] != (qty + 7) // 8 or len(body) != 5 + body[4]:
            raise ModbusError(3)
        with LOCK:
            dev.write("co", start, unpack_bits(body[5:], qty))
        return bytes([fc]) + body[0:4]

    if fc == 0x10:                          # write multiple registers
        if len(body) < 5:
            raise ModbusError(3)
        start = int.from_bytes(body[0:2], "big")
        qty = int.from_bytes(body[2:4], "big")
        if body[4] != qty * 2 or len(body) != 5 + body[4]:
            raise ModbusError(3)
        vals = [int.from_bytes(body[5 + 2 * i:7 + 2 * i], "big") for i in range(qty)]
        with LOCK:
            dev.write("hr", start, vals)
        return bytes([fc]) + body[0:4]

    raise ModbusError(1)                    # illegal function


def handle_pdu(devices, pdu):
    """PDU = адрес + функция + данные. Возвращает ответный PDU или None."""
    addr, fc = pdu[0], pdu[1]
    dev = devices.get(addr)
    if dev is None:
        # Широковещательная запись (адрес 0): применяем ко всем, не отвечаем
        if addr == 0 and fc in (0x05, 0x06, 0x0F, 0x10):
            for d in devices.values():
                try:
                    handle_write(d, fc, pdu[2:])
                except ModbusError:
                    pass
        return None
    try:
        if fc in KIND_OF_FC:
            resp = handle_read(dev, fc, pdu[2:])
        else:
            resp = handle_write(dev, fc, pdu[2:])
        log_frame(dev, fc, pdu[2:])
        return bytes([addr]) + resp
    except ModbusError as e:
        return bytes([addr, fc | 0x80, e.code])


# ---------------------------------------------------------------------------
# Лог на экран — чтобы в курсе было видно, что ходит по шине
# ---------------------------------------------------------------------------

def log_frame(dev, fc, body):
    try:
        if fc in KIND_OF_FC:
            start, qty = int.from_bytes(body[0:2], "big"), int.from_bytes(body[2:4], "big")
            what = "%s #%d..%d" % (KIND_NAMES[KIND_OF_FC[fc]], start, start + qty - 1)
            print("[шина] %s (%d): мастер читает %s" % (dev.name, dev.addr, what))
        elif fc in (0x05, 0x06):
            num = int.from_bytes(body[0:2], "big")
            raw = int.from_bytes(body[2:4], "big")
            kind = "co" if fc == 0x05 else "hr"
            raw = 1 if (fc == 0x05 and raw == 0xFF00) else raw
            reg = find_reg(dev, kind, num)
            human = Device.human_of(reg, raw) if reg else raw
            print("[шина] %s (%d): мастер записал %s #%d = %s (по шине %d)"
                  % (dev.name, dev.addr, KIND_NAMES[kind], num, human, raw))
        else:
            print("[шина] %s (%d): мастер пишет функцией 0x%02X" % (dev.name, dev.addr, fc))
    except Exception:
        pass


def find_reg(dev, kind, num):
    for r in dev.regs:
        if KINDS[r["type"]] == kind and r["num"] == num:
            return r
    return None


# ---------------------------------------------------------------------------
# Транспорт 1: Modbus RTU на последовательном порту
# ---------------------------------------------------------------------------

def open_serial(port, baud):
    """Открываем порт через pyserial; если его нет — через termios (Linux/macOS).
    При любой неудаче бросаем RuntimeError с понятным текстом — молчать нельзя."""
    try:
        import serial
    except ImportError:
        if os.name == "nt":
            raise RuntimeError(
                "не установлен пакет pyserial. Выполните:  pip install pyserial")
        # Запасной путь без pyserial (Linux/macOS)
        import termios, tty
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise RuntimeError("не могу открыть %s: %s" % (port, e))
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        speed = getattr(termios, "B%d" % baud, termios.B9600)
        attrs[4] = attrs[5] = speed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        class PosixSerial:
            timeout = 0
            def read(self, n):
                r, _, _ = select.select([fd], [], [], self.timeout)
                if not r:
                    return b""
                data = os.read(fd, n)
                if data == b"":
                    raise OSError("устройство исчезло с порта")
                return data
            def write(self, b):
                return os.write(fd, b)
        return PosixSerial()

    try:
        return serial.Serial(port, baudrate=baud, bytesize=8,
                             parity="N", stopbits=1, timeout=0)
    except Exception as e:
        raise RuntimeError(
            "не могу открыть %s: %s. Проверьте, что порт есть в Диспетчере "
            "устройств и не занят другой программой (PuTTY, монитор порта "
            "Arduino IDE и т.п.)" % (port, e))


def rtu_loop(devices, port, baud):
    """Надзиратель порта: открыть -> работать -> при потере переоткрыть."""
    addrs = ", ".join(map(str, sorted(devices)))
    while True:
        print("[порт] открываю %s (%d бод)..." % (port, baud), flush=True)
        set_bus(False, "открываю %s..." % port)
        try:
            ser = open_serial(port, baud)
        except RuntimeError as e:
            print("[порт] ОШИБКА: %s" % e, flush=True)
            set_bus(False, str(e))
            time.sleep(3)
            continue
        # Конец кадра в RTU — пауза 3.5 символа (минимум 1.75 мс)
        ser.timeout = max(0.00175, 3.5 * 11 / baud)
        print("[порт] %s открыт, адреса приборов: %s — жду запросы мастера"
              % (port, addrs), flush=True)
        set_bus(True, "%s, %d бод — на связи" % (port, baud))
        try:
            rtu_serve(ser, devices)
        except Exception as e:
            print("[порт] потерян: %s — переоткрою через 2 с" % e, flush=True)
            set_bus(False, "порт %s потерян (%s), жду его возвращения..." % (port, e))
        try:
            ser.close()
        except Exception:
            pass
        time.sleep(2)


def rtu_serve(ser, devices):
    buf = bytearray()
    while True:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            continue
        if not buf:
            continue
        frame, buf = bytes(buf), bytearray()
        if len(frame) < 4:
            continue
        body, got_crc = frame[:-2], int.from_bytes(frame[-2:], "little")
        if crc16(body) != got_crc:
            print("[шина] битый кадр (CRC):", frame.hex(" "), flush=True)
            continue
        resp = handle_pdu(devices, body)
        if resp:
            ser.write(resp + crc16(resp).to_bytes(2, "little"))


# ---------------------------------------------------------------------------
# Транспорт 2: Modbus TCP (для отладки без железа)
# ---------------------------------------------------------------------------

def tcp_client(conn, devices):
    try:
        while True:
            head = recv_exact(conn, 7)
            if not head:
                return
            tid, pid, length, unit = head[0:2], head[2:4], int.from_bytes(head[4:6], "big"), head[6]
            pdu = recv_exact(conn, length - 1)
            if pdu is None:
                return
            resp = handle_pdu(devices, bytes([unit]) + pdu)
            if resp:
                body = resp[1:]
                conn.sendall(tid + pid + (len(body) + 1).to_bytes(2, "big") + bytes([unit]) + body)
    except OSError:
        pass
    finally:
        conn.close()


def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def tcp_loop(devices, tcp_port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", tcp_port))
    except OSError as e:
        print("[tcp] ОШИБКА: не могу занять порт %d: %s" % (tcp_port, e), flush=True)
        set_bus(False, "TCP-порт %d занят или недоступен" % tcp_port)
        return
    srv.listen(5)
    print("[tcp] Modbus TCP: порт %d, адреса: %s"
          % (tcp_port, ", ".join(map(str, sorted(devices)))), flush=True)
    set_bus(True, "Modbus TCP: порт %d — на связи" % tcp_port)
    while True:
        conn, _ = srv.accept()
        threading.Thread(target=tcp_client, args=(conn, devices), daemon=True).start()


# ---------------------------------------------------------------------------
# Веб-панель
# ---------------------------------------------------------------------------

def build_state(devices):
    out = {"devices": [], "bus": dict(BUS_STATUS)}
    with LOCK:
        for dev in devices.values():
            d = {"addr": dev.addr, "name": dev.name, "image": dev.image, "regs": []}
            for r in dev.regs:
                kind = KINDS[r["type"]]
                raw = dev.values[kind][r["num"]]
                row = dict(r)  # type, num, widget, label, unit, min, max, scale
                row["raw"] = raw
                row["value"] = Device.human_of(r, raw)
                d["regs"].append(row)
            out["devices"].append(d)
    return out


def make_http_handler(devices):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # не засоряем консоль: важен лог шины, а не HTTP

        def _send(self, code, body, ctype="text/html; charset=utf-8"):
            data = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/api/state":
                self._send(200, json.dumps(build_state(devices), ensure_ascii=False),
                           "application/json; charset=utf-8")
                return
            path = "/index.html" if self.path == "/" else self.path
            # Отдаём файлы только из каталога скрипта, без выхода «вверх»
            target = (HERE / path.lstrip("/")).resolve()
            if target.is_file() and HERE in target.parents:
                ctype = {".html": "text/html; charset=utf-8",
                         ".svg": "image/svg+xml",
                         ".json": "application/json; charset=utf-8"}.get(target.suffix,
                                                                          "application/octet-stream")
                self._send(200, target.read_bytes(), ctype)
            else:
                self._send(404, "not found", "text/plain; charset=utf-8")

        def do_POST(self):
            if self.path != "/api/set":
                self._send(404, "not found", "text/plain; charset=utf-8")
                return
            try:
                req = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                dev = devices[req["addr"]]
                # Человек — это среда: руками можно задавать только измеряемые
                # величины (input/discrete). Holding и coil пишет только мастер.
                if req["kind"] not in ("input", "discrete"):
                    raise ValueError("этим управляет мастер, а не среда")
                reg = find_reg(dev, KINDS[req["kind"]], req["num"])
                if reg is None:
                    raise KeyError("reg")
                with LOCK:
                    if reg.get("widget") == "bitfield":
                        dev.set_bit(reg, int(req["bit"]), bool(req["value"]))
                    else:
                        dev.set_human(reg, req["value"])
                self._send(200, '{"ok": true}', "application/json")
            except (KeyError, ValueError, json.JSONDecodeError) as e:
                self._send(400, '{"ok": false, "error": "%s"}' % e, "application/json")
    return Handler


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Виртуальные Modbus-приборы")
    ap.add_argument("--port", help="COM-порт для Modbus RTU (например COM3 или /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=9600, help="скорость (по умолчанию 9600)")
    ap.add_argument("--tcp", type=int, metavar="PORT", help="вместо COM — Modbus TCP на порту")
    ap.add_argument("--config", default=str(HERE / "devices.json"), help="файл описания приборов")
    ap.add_argument("--http", type=int, default=8080, help="порт веб-панели (по умолчанию 8080)")
    args = ap.parse_args()

    if not args.port and args.tcp is None:
        ap.error("укажите --port COM3 для RTU или --tcp 502 для TCP")

    devices = load_devices(args.config)

    if args.tcp is not None:
        threading.Thread(target=tcp_loop, args=(devices, args.tcp), daemon=True).start()
    else:
        threading.Thread(target=rtu_loop, args=(devices, args.port, args.baud), daemon=True).start()

    httpd = ThreadingHTTPServer(("127.0.0.1", args.http), make_http_handler(devices))
    print("Веб-панель: http://127.0.0.1:%d  (Ctrl+C — выход)" % args.http)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
