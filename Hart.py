# https://pypi.org/project/hart-protocol/

import io
import os
import json
import time
import struct
import traceback
import threading
from tango import AttrWriteType, AttrDataFormat, DevState, Attr, SpectrumAttr, ImageAttr, CmdArgType, UserDefaultAttrProp
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
    init_dynamic_attributes = device_property(dtype=str, default_value="")

    # ───────────── Lifecycle ─────────────
    def init_device(self):
        self.set_state(DevState.INIT)
        self.get_device_properties(self.get_device_class())
        self.dynamicAttributes = {}
        self.dynamicAttributeLookup = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
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

    def delete_device(self):
        if hasattr(self, "_stop_event"):
            self._stop_event.set()

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


if __name__ == "__main__":
    server_name = os.getenv("DEVICE_SERVER_NAME", "Hart")
    run({server_name: Hart})
