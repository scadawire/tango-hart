# https://pypi.org/project/hart-protocol/

import io
import os
import sys
import json
import time
import struct
import traceback
import threading
from tango import AttrWriteType, AttrDataFormat, DevState, Attr, SpectrumAttr, ImageAttr, CmdArgType, UserDefaultAttrProp, EnsureOmniThread
from tango.server import Device, attribute, command, DeviceMeta, device_property, run
# hart_protocol is imported lazily inside connect() so the module can be loaded
# (and tested) without the library installed.


class _BytesBuffer:
    """Wraps a bytes object to provide the in_waiting/read interface Unpacker needs."""

    def __init__(self, data):
        self._buf = io.BytesIO(data)

    @property
    def in_waiting(self):
        pos = self._buf.tell()
        self._buf.seek(0, 2)
        end = self._buf.tell()
        self._buf.seek(pos)
        return end - pos

    def read(self, n=1):
        return self._buf.read(n)


# HART command IDs that support writes (master -> device)
_WRITE_COMMANDS = {6, 17, 18, 19}


class Hart(Device, metaclass=DeviceMeta):

    # ───────────── Device Properties ─────────────
    serial_port = device_property(dtype=str, default_value="/dev/ttyUSB0")
    baudrate    = device_property(dtype=int,   default_value=1200)
    parity      = device_property(dtype=str,   default_value="O")
    address     = device_property(dtype=int,   default_value=0)
    poll_interval = device_property(dtype=float, default_value=1.0)
    timeout     = device_property(dtype=float,  default_value=0.5)
    health_check_interval = device_property(dtype=float, default_value=2.0)
    init_dynamic_attributes = device_property(dtype=str, default_value="")

    # ───────────── Lifecycle ─────────────
    def init_device(self):
        self.set_state(DevState.INIT)
        self.get_device_properties(self.get_device_class())
        self.dynamicAttributes = {}
        self.dynamicAttributeLookup = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._health_stop = None
        self._serial = None
        self.last_error = ""

        self.connect()

        if self.init_dynamic_attributes:
            try:
                attrs = json.loads(self.init_dynamic_attributes)
                for a in attrs:
                    self.add_dynamic_attribute(
                        a["name"],
                        a.get("data_type", ""),
                        a.get("min_value", ""),
                        a.get("max_value", ""),
                        a.get("unit", ""),
                        a.get("write_type", ""),
                        a.get("label", ""),
                        a.get("register", ""),
                        a.get("data_format", ""),
                        str(a.get("max_x", "")),
                        str(a.get("max_y", "")),
                    )
            except Exception:
                for name in self.init_dynamic_attributes.split(","):
                    self.add_dynamic_attribute(name.strip())

        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        if self.get_state() != DevState.FAULT:
            self.set_state(DevState.ON)
        self.start_health_monitor()

    def delete_device(self):
        if hasattr(self, "_stop_event"):
            self._stop_event.set()
        if getattr(self, "_health_stop", None) is not None:
            self._health_stop.set()

    # ───────────── Health Monitor ─────────────
    # A HART device is polled, never subscribed to, and the poll loop swallows what it cannot read -
    # a device that stopped answering would leave every attribute at its last value with the driver
    # still in ON. So the device is watched actively.
    #
    # The probe is command 0, read_unique_identifier: it is universal, so it works whatever the
    # attributes happen to map, and it is what a master identifies a device with in the first place.
    # Nothing has to be reopened on recovery - the serial port belongs to the line, not to the device
    # at the far end of it, so the driver returns to ON as soon as the device answers again.
    def health_probe(self):
        if self._serial is None:
            raise Exception("the serial line is not open")
        if self._send_command(self.address, 0) is None:
            raise Exception("no answer to command 0 from device " + str(self.address))

    def _health_loop(self):
        with EnsureOmniThread():
            healthy = True
            while not self._health_stop.is_set():
                self._health_stop.wait(self.health_check_interval)
                if self._health_stop.is_set():
                    break
                try:
                    self.health_probe()
                    if not healthy:
                        self.info_stream("HART device %s answers again", self.address)
                        self.set_state(DevState.ON)
                        healthy = True
                except Exception as e:
                    if healthy:
                        self.warn_stream("lost the HART device %s: %s", self.address, str(e))
                        self.set_state(DevState.FAULT)
                        healthy = False

    def start_health_monitor(self):
        self._health_stop = threading.Event()
        self._health_thread = threading.Thread(target=self._health_loop, daemon=True)
        self._health_thread.start()

    # ───────────── Connection ─────────────
    def connect(self):
        try:
            import serial
            self._serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                parity=self.parity,
                stopbits=1,
                bytesize=8,
                timeout=self.timeout,
            )
            self.info_stream("Connected to HART device on %s", self.serial_port)
        except Exception as e:
            self.last_error = str(e)
            self.error_stream("%s", traceback.format_exc())
            self.set_state(DevState.FAULT)

    # ───────────── Poll Loop ─────────────
    def _poll_loop(self):
        while not self._stop_event.is_set():
            try:
                self._run_poll()
            except Exception as e:
                self.warn_stream("poll error: %s", str(e))
            self._stop_event.wait(timeout=self.poll_interval)

    def _run_poll(self):
        if self._serial is None:
            return

        # Collect unique cmd_ids in registration order
        seen = set()
        cmd_ids = []
        for lookup in self.dynamicAttributeLookup.values():
            cid = lookup["cmd_id"]
            if cid not in seen:
                seen.add(cid)
                cmd_ids.append(cid)

        for cmd_id in cmd_ids:
            response = self._send_command(self.address, cmd_id)
            if response is None:
                continue
            for name, lookup in self.dynamicAttributeLookup.items():
                if lookup["cmd_id"] == cmd_id:
                    field = lookup["field_name"]
                    if hasattr(response, field):
                        raw = getattr(response, field)
                        self.dynamicAttributes[name] = self._cast_value(raw, lookup["variableType"])

    def _send_command(self, device_addr, cmd_id, data=None):
        from hart_protocol.tools import pack_command
        from hart_protocol import Unpacker

        cmd_bytes = pack_command(device_addr, cmd_id, data)
        with self._lock:
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass
            self._serial.write(cmd_bytes)

            response_data = b""
            deadline = time.time() + self.timeout
            while time.time() < deadline:
                n = self._serial.in_waiting
                if n > 0:
                    response_data += self._serial.read(n)
                elif response_data:
                    break
                time.sleep(0.005)

        if not response_data:
            return None

        buf = _BytesBuffer(response_data)
        up = Unpacker(buf)
        try:
            return next(up)
        except StopIteration:
            return None

    # ───────────── Dynamic Attributes ─────────────
    def add_dynamic_attribute(
        self, name, variable_type_name="",
        min_value="", max_value="", unit="",
        write_type_name="", label="", register="",
        data_format_name="", max_x="", max_y=""
    ):
        if not name:
            return

        prop = UserDefaultAttrProp()
        var_type = self.stringValueToVarType(variable_type_name)
        reg = self.parse_register(register)
        data_format = self.stringValueToFormatType(data_format_name)
        dim_x = int(max_x) if max_x else 256
        dim_y = int(max_y) if max_y else 256

        # Write commands are READ_WRITE by default; read commands are READ
        if write_type_name == "":
            effective_wt = "READ_WRITE" if reg["cmd_id"] in _WRITE_COMMANDS else "READ"
        else:
            effective_wt = write_type_name
        write_type = self.stringValueToWriteType(effective_wt)

        if unit:
            prop.set_unit(unit)
        if label:
            prop.set_label(label)
        if min_value:
            prop.set_min_value(min_value)
        if max_value:
            prop.set_max_value(max_value)

        if data_format == AttrDataFormat.SPECTRUM:
            attr = SpectrumAttr(name, var_type, write_type, dim_x)
        elif data_format == AttrDataFormat.IMAGE:
            attr = ImageAttr(name, var_type, write_type, dim_x, dim_y)
        else:
            attr = Attr(name, var_type, write_type)
        attr.set_default_properties(prop)

        self.dynamicAttributeLookup[name] = {
            "variableType": var_type,
            "cmd_id": reg["cmd_id"],
            "field_name": reg["field_name"],
            "dataFormat": data_format,
            "max_x": dim_x,
            "max_y": dim_y,
        }
        self.dynamicAttributes[name] = self._default_value(var_type)
        self.add_attribute(attr, r_meth=self.read_dynamic_attr, w_meth=self.write_dynamic_attr)
        self.set_change_event(name, True, False)

    # ───────────── Attribute Access ─────────────
    def read_dynamic_attr(self, attr):
        name = attr.get_name()
        value = self.dynamicAttributes[name]
        attr.set_value(value)

    def write_dynamic_attr(self, attr):
        name = attr.get_name()
        value = attr.get_write_value()
        lookup = self.dynamicAttributeLookup[name]
        cmd_id = lookup["cmd_id"]
        var_type = lookup["variableType"]

        data = self._build_write_data(cmd_id, lookup["field_name"], value, var_type)
        if data is None:
            self.warn_stream("write not supported for HART command %s", cmd_id)
            return

        response = self._send_command(self.address, cmd_id, data)
        if response is not None:
            field = lookup["field_name"]
            if hasattr(response, field):
                self.dynamicAttributes[name] = self._cast_value(
                    getattr(response, field), var_type
                )
                self.push_change_event(name, self.dynamicAttributes[name])

    def _build_write_data(self, cmd_id, field_name, value, var_type):
        """Return packed data bytes for a write command, or None if unsupported."""
        from hart_protocol import tools
        if cmd_id == 6:
            # CMD 6: write polling address (0-15)
            return int(value).to_bytes(1, "big")
        elif cmd_id == 17:
            # CMD 17: write message (32-char ASCII)
            msg = str(value).ljust(32)[:32]
            return tools.pack_ascii(msg)
        elif cmd_id == 19:
            # CMD 19: write final assembly number (24-bit)
            return int(value).to_bytes(3, "big")
        return None

    # ───────────── Register Parsing ─────────────
    def parse_register(self, register):
        try:
            parts = register.split(".", 1)
            cmd_id = int(parts[0])
            field_name = parts[1] if len(parts) > 1 else ""
            if not field_name:
                raise ValueError("field_name missing")
            return {"cmd_id": cmd_id, "field_name": field_name}
        except (ValueError, IndexError):
            raise Exception(
                "Invalid register descriptor '" + register + "', "
                "expected: cmd_id.field_name  e.g. '1.primary_variable'"
            )

    # ───────────── Type Helpers ─────────────
    def _cast_value(self, raw, var_type):
        """Cast a raw HART response field value to the requested Tango type."""
        if var_type == CmdArgType.DevFloat:
            return float(raw)
        elif var_type == CmdArgType.DevDouble:
            return float(raw)
        elif var_type == CmdArgType.DevLong:
            return int(raw)
        elif var_type == CmdArgType.DevShort:
            return int(raw)
        elif var_type == CmdArgType.DevLong64:
            return int(raw)
        elif var_type == CmdArgType.DevBoolean:
            return bool(raw)
        elif var_type == CmdArgType.DevString:
            if isinstance(raw, (bytes, bytearray)):
                return raw.decode("utf-8", errors="replace")
            return str(raw)
        raise Exception("Unsupported variable type: " + str(var_type))

    def _default_value(self, var_type):
        return {
            CmdArgType.DevFloat:   0.0,
            CmdArgType.DevDouble:  0.0,
            CmdArgType.DevLong:    0,
            CmdArgType.DevShort:   0,
            CmdArgType.DevLong64:  0,
            CmdArgType.DevBoolean: False,
            CmdArgType.DevString:  "",
        }.get(var_type, "")

    def stringValueToVarType(self, name):
        return {
            "DevBoolean": CmdArgType.DevBoolean,
            "DevShort":   CmdArgType.DevShort,
            "DevLong":    CmdArgType.DevLong,
            "DevFloat":   CmdArgType.DevFloat,
            "DevDouble":  CmdArgType.DevDouble,
            "DevLong64":  CmdArgType.DevLong64,
            "DevString":  CmdArgType.DevString,
            "":           CmdArgType.DevDouble,
        }.get(name, CmdArgType.DevDouble)

    def stringValueToWriteType(self, name):
        return {
            "READ":       AttrWriteType.READ,
            "WRITE":      AttrWriteType.WRITE,
            "READ_WRITE": AttrWriteType.READ_WRITE,
            "":           AttrWriteType.READ,
        }.get(name, AttrWriteType.READ)

    def stringValueToFormatType(self, name):
        return {
            "SCALAR":   AttrDataFormat.SCALAR,
            "SPECTRUM": AttrDataFormat.SPECTRUM,
            "IMAGE":    AttrDataFormat.IMAGE,
        }.get(name, AttrDataFormat.SCALAR)


# ───────────── Device Scan ─────────────
# "Hart --scan '<json>'" looks for field devices instead of running the device server, so scadawire can
# offer what it finds as devices to create. It prints one line, SCAN_RESULT_MARKER followed by
# {"devices": [{"name", "properties", "details", "issues", "notes"}], "error"}: "properties" are named
# like this driver's device properties, so a found device is configured by copying them over. "issues"
# are what speaks against using a found device, "notes" what has to be done before it works.
SCAN_RESULT_MARKER = "SCADAWIRE_SCAN_RESULT "

# Short frame, master to slave: the polling address in the low 6 bits of a single address byte, the
# primary master bit set. Command 0 is the one command a field device answers by its polling address -
# it is how a master learns the long address everything else is sent to.
_SHORT_FRAME_REQUEST = 0x02
_PRIMARY_MASTER = 0x80


def _short_frame(polling_address, command_id):
    body = bytes([_SHORT_FRAME_REQUEST, _PRIMARY_MASTER | (polling_address & 0x3F), command_id, 0x00])
    checksum = 0
    for b in body:
        checksum ^= b
    return b"\xFF" * 5 + body + bytes([checksum])


def _transact(line, frame, timeout):
    try:
        line.reset_input_buffer()
    except Exception:
        pass
    line.write(frame)
    response = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = line.in_waiting
        if n > 0:
            response += line.read(n)
        elif response:
            break
        time.sleep(0.005)
    return response


def _open_line(open_fn, serial_port, reset=None):
    """open_fn(), working around what a pty does when it is set to the settings it already has.

    A pty left at exactly these settings by the last one to open it - the device server of the line,
    a scan before - answers tcsetattr with EINVAL, and the open fails. Only the settings changing makes
    it accept them again, so a failed open is retried once behind an open at other settings, as the
    hart simulator does. A real serial port never does this; every SITL and demo line is a pty.
    """
    import termios
    try:
        return open_fn()
    except termios.error:
        if reset is None:
            import serial
            reset = lambda: serial.Serial(port=serial_port, baudrate=9600, parity="N", timeout=0).close()
        reset()
        return open_fn()


def scan(config, line=None):
    """Polls every polling address of the line with command 0, read unique identifier.

    HART 5 devices take polling addresses 0 to 15, HART 6 and 7 ones up to 63 - max_address says how far
    to look, every address without a device costs the timeout. The line must not be in use by a running
    device server meanwhile, its traffic would interleave with the scan's.
    """
    from hart_protocol import Unpacker
    from hart_protocol.tools import calculate_long_address
    serial_port = str(config.get("serial_port") or "/dev/ttyUSB0")
    baudrate = int(config.get("baudrate") or 1200)
    parity = str(config.get("parity") or "O")
    max_address = config.get("max_address")
    max_address = min(63, max(0, 15 if max_address in (None, "") else int(max_address)))
    timeout = float(config.get("timeout") or 0.5)
    ownLine = line is None
    if ownLine:
        import serial
        line = _open_line(lambda: serial.Serial(port=serial_port, baudrate=baudrate, parity=parity,
            stopbits=1, bytesize=8, timeout=timeout), serial_port)
    try:
        found = []
        for address in range(0, max_address + 1):
            raw = _transact(line, _short_frame(address, 0), timeout)
            if not raw:
                continue
            try:
                reply = next(Unpacker(_BytesBuffer(raw)))
            except Exception:
                continue
            if getattr(reply, "command", None) != 0 or not hasattr(reply, "manufacturer_id"):
                continue
            found.append((address, reply))
        devices = []
        for address, reply in found:
            notes = []
            if len(found) > 1:
                notes.append("the line carries %d devices and every HART device server opens the serial port "
                    "of its own, so only one device of this line can run at a time" % len(found))
            longAddress = calculate_long_address(reply.manufacturer_id, reply.manufacturer_device_type,
                int(reply.device_id).to_bytes(3, "big"))
            devices.append({
                "name": "HART device %d (manufacturer 0x%02x, type 0x%02x) on %s"
                    % (address, reply.manufacturer_id, reply.manufacturer_device_type, serial_port),
                "properties": {"serial_port": serial_port, "baudrate": baudrate, "parity": parity, "address": address},
                "details": {
                    "polling_address": address,
                    "manufacturer_id": "0x%02x" % reply.manufacturer_id,
                    "device_type": "0x%02x" % reply.manufacturer_device_type,
                    "device_id": reply.device_id,
                    "long_address": longAddress.hex(),
                    "universal_revision": reply.universal_command_revision_level,
                    "device_revision": reply.transmitter_specific_command_revision_level,
                    "software_revision": reply.software_revision_level,
                    "hardware_revision": reply.hardware_revision_level,
                },
                "issues": [],
                "notes": notes,
            })
        return {"devices": devices}
    finally:
        if ownLine:
            line.close()


def scan_main(argv):
    try:
        config = json.loads(argv[argv.index("--scan") + 1]) if len(argv) > argv.index("--scan") + 1 else {}
        result = scan(config)
    except Exception as e:
        result = {"devices": [], "error": "%s: %s" % (type(e).__name__, e)}
    print(SCAN_RESULT_MARKER + json.dumps(result), flush=True)
    return 1 if result.get("error") else 0


if __name__ == "__main__":
    if "--scan" in sys.argv:
        sys.exit(scan_main(sys.argv))
    server_name = os.getenv("DEVICE_SERVER_NAME", "Hart")
    run({server_name: Hart})
