import sys
import time
import threading
import ctypes
from enum import IntEnum
from typing import Optional, List, Callable
import serial
from serial.tools import list_ports

# wireview-pro-2.py
# GitHub Copilot
# Read data from Thermal Grizzly WireView Pro II (USB CDC ACM, VID=0x0483, PID=0x5740)
# Requires: pip install pyserial

USB_VID = 0x0483
USB_PID = 0x5740

WELCOME = "Thermal Grizzly WireView Pro II"

# USB command codes (match firmware/C#)
class UsbCmd(IntEnum):
    WELCOME = 0x00
    READ_VENDOR_DATA = 0x01
    READ_UID = 0x02
    READ_DEVICE_DATA = 0x03
    READ_SENSOR_VALUES = 0x04
    READ_CONFIG = 0x05
    WRITE_CONFIG = 0x06
    READ_CALIBRATION = 0x07
    WRITE_CALIBRATION = 0x08
    SPI_FLASH_WRITE_PAGE = 0x09
    SPI_FLASH_READ_PAGE = 0x0A
    SPI_FLASH_ERASE_SECTOR = 0x0B
    SCREEN_CHANGE = 0x0C
    READ_BUILD_INFO = 0x0D
    CLEAR_FAULTS = 0x0E
    RESET = 0xF0
    BOOTLOADER = 0xF1
    NVM_CONFIG = 0xF2
    NOP = 0xFF


class ScreenCmd(IntEnum):
    GOTO_MAIN = 0xE0
    GOTO_SIMPLE = 0xE1
    GOTO_CURRENT = 0xE2
    GOTO_TEMP = 0xE3
    GOTO_STATUS = 0xE4
    GOTO_SAME = 0xEF
    PAUSE_UPDATES = 0xF0
    RESUME_UPDATES = 0xF1


class Fault(IntEnum):
    OTP_TCHIP = 0
    OTP_TS = 1
    OCP = 2
    WIRE_OCP = 3
    OPP = 4
    CURRENT_IMBALANCE = 5
    NUM = 6

# Struct sizes with Pack=4 (match C# Marshal.SizeOf)
DEVICE_STR_LEN = 32

class VendorData(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("vendor_id", ctypes.c_uint8),
        ("product_id", ctypes.c_uint8),
        ("fw_version", ctypes.c_uint8),
    ]


class BuildInfo(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("vendor", VendorData),
        ("product_name_raw", ctypes.c_char * DEVICE_STR_LEN),
        ("build_info_raw", ctypes.c_char * DEVICE_STR_LEN),
        ("product_name_length", ctypes.c_uint8),
    ]

    @property
    def product_name(self) -> str:
        return bytes(self.product_name_raw).split(b"\x00", 1)[0].decode("ascii", errors="ignore")

    @property
    def build_info(self) -> str:
        return bytes(self.build_info_raw).split(b"\x00", 1)[0].decode("ascii", errors="ignore")


class PowerSensor(ctypes.LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("voltage_mV", ctypes.c_int16),
        ("_pad0", ctypes.c_uint16),
        ("current_mA", ctypes.c_uint32),
        ("power_mW", ctypes.c_uint32),
    ]


class SensorValues(ctypes.LittleEndianStructure):
    _pack_ = 4
    _fields_ = [
        ("ts_x10_c", ctypes.c_int16 * 4),
        ("vdd_mV", ctypes.c_uint16),
        ("fan_duty_pct", ctypes.c_uint8),
        ("_pad1", ctypes.c_uint8),
        ("pins", PowerSensor * 6),
        ("total_power_mW", ctypes.c_uint32),
        ("total_current_mA", ctypes.c_uint32),
        ("avg_voltage_mV", ctypes.c_uint16),
        ("hpwr_capability", ctypes.c_uint8),
        ("_pad2", ctypes.c_uint8),
        ("fault_status", ctypes.c_uint16),
        ("fault_log", ctypes.c_uint16),
    ]

    @property
    def ts_in_c(self) -> float:
        return self.ts_x10_c[0] / 10.0

    @property
    def ts_out_c(self) -> float:
        return self.ts_x10_c[1] / 10.0

    @property
    def ts3_c(self) -> float:
        return self.ts_x10_c[2] / 10.0

    @property
    def ts4_c(self) -> float:
        return self.ts_x10_c[3] / 10.0


def _fmt_v_from_mV(mV: int) -> str:
    return f"{(float(mV) / 1000.0):.3f} V"


def _fmt_a_from_mA(mA: int) -> str:
    return f"{(float(mA) / 1000.0):.3f} A"


def _fmt_w_from_mW(mW: int) -> str:
    return f"{(float(mW) / 1000.0):.3f} W"


def _render_sensor_table(sv: SensorValues) -> str:
    ts_line = (
        f"TsIn {sv.ts_in_c:6.1f} °C  "
        f"TsOut {sv.ts_out_c:6.1f} °C  "
        f"Ts3 {sv.ts3_c:6.1f} °C  "
        f"Ts4 {sv.ts4_c:6.1f} °C  "
        f"Fan {int(sv.fan_duty_pct):3d} %  "
        f"Vavg {_fmt_v_from_mV(int(sv.avg_voltage_mV))}  "
        f"Itot {_fmt_a_from_mA(int(sv.total_current_mA))}  "
        f"Ptot {_fmt_w_from_mW(int(sv.total_power_mW))}"
    )

    lines = [
        ts_line,
        "",
        "Pin  |    Voltage |    Current |     Power",
        "-----+------------+------------+----------",
    ]
    for i, p in enumerate(sv.pins, start=1):
        lines.append(
            f"{i:>3d}  | {(_fmt_v_from_mV(int(p.voltage_mV))):>10s} | {(_fmt_a_from_mA(int(p.current_mA))):>10s} | {(_fmt_w_from_mW(int(p.power_mW))):>8s}"
        )

    lines.append("")

    fault_status = int(sv.fault_status)
    fault_log = int(sv.fault_log)
    lines.append(f"HPWR {int(sv.hpwr_capability)}  FaultStatus 0x{fault_status:04X}  FaultLog 0x{fault_log:04X}")
    lines.append("Fault             | Status | Log")
    lines.append("------------------+--------+-----")
    for f in Fault:
        if f is Fault.NUM:
            continue
        s = 1 if (fault_status >> int(f)) & 1 else 0
        l = 1 if (fault_log >> int(f)) & 1 else 0
        lines.append(f"{f.name:<18} |   {s:d}    |  {l:d}")

    return "\n".join(lines)


VENDOR_DATA_SIZE = ctypes.sizeof(VendorData)
BUILD_INFO_SIZE = ctypes.sizeof(BuildInfo)
POWER_SENSOR_SIZE = ctypes.sizeof(PowerSensor)
SENSOR_SIZE = ctypes.sizeof(SensorValues)

def find_wireview_ports() -> List[str]:
    ports = []
    for p in list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            ports.append(p.device)
    return ports

class WireViewPro2:
    def __init__(self, port: Optional[str] = None, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.hardware_revision = ""
        self.firmware_version = ""
        self.unique_id = ""
        self.poll_interval_ms = 1000
        self._poll_thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self.on_data: Optional[Callable[[SensorValues], None]] = None
        self._config_version = 0  # 0=V1, 1=V2

    def connect(self) -> bool:
        if self.connected:
            return True
        if not self.port:
            matches = find_wireview_ports()
            if not matches:
                return False
            self.port = matches[0]
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1, write_timeout=1, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
            # Read welcome via RTS pulse, no command
            if not self._read_welcome():
                self._cleanup_serial()
                return False

            vd = self.read_vendor_data()
            if vd and vd.vendor_id == 0xEF and vd.product_id == 0x05:
                self.hardware_revision = f"{vd.vendor_id:02X}{vd.product_id:02X}"
                self.firmware_version = str(vd.fw_version)
                self._config_version = 1 if vd.fw_version > 2 else 0

                uid = self.read_uid()
                if uid:
                    self.unique_id = uid

                # Resume display updates
                self.screen_cmd(ScreenCmd.RESUME_UPDATES)

                self.connected = True
                # Start polling thread
                self._stop_evt.clear()
                self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
                self._poll_thread.start()
                return True
        except Exception:
            self._cleanup_serial()
            return False
        return False

    def disconnect(self):
        if not self.connected:
            self._cleanup_serial()
            return
        try:
            self._stop_evt.set()
            if self._poll_thread and self._poll_thread.is_alive():
                self._poll_thread.join(timeout=1.0)
        except Exception:
            pass
        self.connected = False
        self.hardware_revision = ""
        self.firmware_version = ""
        self.unique_id = ""
        self._cleanup_serial()

    # ---- High-level reads ----

    def read_build_info(self) -> Optional[BuildInfo]:
        buf = self._send_cmd(UsbCmd.READ_BUILD_INFO, BUILD_INFO_SIZE)
        if not buf or len(buf) != BUILD_INFO_SIZE:
            return None
        return BuildInfo.from_buffer_copy(buf)

    def read_vendor_data(self) -> Optional[VendorData]:
        buf = self._send_cmd(UsbCmd.READ_VENDOR_DATA, VENDOR_DATA_SIZE)
        if not buf or len(buf) != VENDOR_DATA_SIZE:
            return None
        return VendorData.from_buffer_copy(buf)

    def read_uid(self) -> Optional[str]:
        buf = self._send_cmd(UsbCmd.READ_UID, 12)
        if not buf or len(buf) != 12:
            return None
        return buf.hex().upper()

    def read_sensor_values(self) -> Optional[SensorValues]:
        buf = self._send_cmd(UsbCmd.READ_SENSOR_VALUES, SENSOR_SIZE)
        if not buf or len(buf) != SENSOR_SIZE:
            return None
        return SensorValues.from_buffer_copy(buf)

    # ---- Commands ----

    def screen_cmd(self, cmd: ScreenCmd | int) -> None:
        self._send_data(bytes([UsbCmd.SCREEN_CHANGE, int(cmd)]), response_size=0)

    def clear_faults(self, fault_status_mask: int = 0xFFFF, fault_log_mask: int = 0xFFFF) -> None:
        data = bytes([
            UsbCmd.CLEAR_FAULTS,
            fault_status_mask & 0xFF, (fault_status_mask >> 8) & 0xFF,
            fault_log_mask & 0xFF, (fault_log_mask >> 8) & 0xFF
        ])
        self._send_data(data, response_size=0)

    def enter_bootloader(self) -> None:
        self._send_cmd(UsbCmd.BOOTLOADER, response_size=0)

    # ---- Internal I/O ----

    def _poll_loop(self):
        try:
            while not self._stop_evt.is_set():
                sv = self.read_sensor_values()
                if sv and self.on_data:
                    try:
                        self.on_data(sv)
                    except Exception:
                        pass
                time.sleep(max(0.1, self.poll_interval_ms / 1000.0))
        except Exception:
            self.disconnect()

    def _read_welcome(self) -> bool:
        if not self.ser:
            return False
        size = len(WELCOME) + 1
        buf = self._send_data(bytes(), response_size=32, rts=True)
        if not buf or len(buf) != size:
            return False
        msg = buf.rstrip(b"\x00").decode("ascii", errors="ignore")
        return msg == WELCOME

    def _send_cmd(self, cmd: int, response_size: int = 0, rts: bool = False) -> Optional[bytes]:
        return self._send_data(bytes([cmd]), response_size=response_size, rts=rts)

    def _send_data(self, data: bytes, response_size: int = 0, rts: bool = False) -> Optional[bytes]:
        if not self.ser:
            return None
        try:
            if rts:
                self.ser.dtr = True
            else:
                self.ser.reset_input_buffer()
            if data:
                self.ser.write(data)
            buf = b""
            if response_size > 0:
                buf = self._read_exact(response_size, timeout_ms=1000) or b""
            if rts:
                self.ser.dtr = False
            return buf if len(buf) == response_size or response_size == 0 else None
        except Exception:
            return None

    def _read_exact(self, size: int, timeout_ms: int = 1000) -> Optional[bytes]:
        if not self.ser:
            return None
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        out = bytearray()
        while len(out) < size and time.monotonic() < deadline:
            n = self.ser.in_waiting
            if n <= 0:
                time.sleep(0.001)
                continue
            chunk = self.ser.read(min(size - len(out), n))
            if not chunk:
                time.sleep(0.001)
                continue
            out.extend(chunk)
        return bytes(out) if len(out) == size else None

    def _cleanup_serial(self):
        try:
            if self.ser:
                try:
                    self.ser.dtr = False
                except Exception:
                    pass
                try:
                    self.ser.close()
                except Exception:
                    pass
        finally:
            self.ser = None

# Example usage: read and print sensor values
def main():
    dev = WireViewPro2()

    def _print_table(sv: SensorValues) -> None:
        if sys.stdout.isatty():
            sys.stdout.write("\x1b[2J\x1b[H")
        print(_render_sensor_table(sv), flush=True)

    dev.on_data = _print_table
    if not dev.connect():
        print("Failed to connect to WireView Pro II (check USB VID/PID 0483:5740)")
        sys.exit(1)
    print(f"Connected: HW={dev.hardware_revision}, FW={dev.firmware_version}, UID={dev.unique_id}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        dev.disconnect()

if __name__ == "__main__":
    main()