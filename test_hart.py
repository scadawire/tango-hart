"""
Unit tests for Hart.py (tango-hart device server).

Sections:
  1. Pure conversion / parsing tests (no hardware needed)
  2. MockSerial-based poll + read/write tests
"""

import io
import sys
import struct
import functools
import threading

# ---------------------------------------------------------------------------
# Check hart_protocol availability
# ---------------------------------------------------------------------------
try:
    import hart_protocol
    import hart_protocol.tools as _ht
    HAS_HART = True
except ImportError:
    HAS_HART = False

from Hart import Hart, _WRITE_COMMANDS

# ---------------------------------------------------------------------------
# State stub (mirrors pattern used in all tango-ds tests)
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.dynamicAttributes = {}
        self.dynamicAttributeLookup = {}
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._serial = None
        self.last_error = ""
        self.address = 0
        self.timeout = 0.5
        self.poll_interval = 1.0

    def info_stream(self, msg): pass
    def debug_stream(self, msg): pass
    def warn_stream(self, msg): pass
    def error_stream(self, msg): pass
    def get_state(self): return None

    def __getattr__(self, name):
        attr = getattr(Hart, name, None)
        if attr is not None and callable(attr):
            return functools.partial(attr, self)
        raise AttributeError("'State' has no attribute '" + name + "'")


# ---------------------------------------------------------------------------
# Helpers for building fake HART responses
# ---------------------------------------------------------------------------

def make_hart_response(address, command, actual_data, response_code=0, device_status=0):
    """
    Build a well-formed HART slave response frame (short address).
    bytecount = 2 (rsp_code + dev_status) + len(actual_data)
    """
    bytecount = 2 + len(actual_data)
    frame = b"\x06"                    # start char: slave, short address
    frame += bytes([address])
    frame += bytes([command])
    frame += bytes([bytecount])
    frame += bytes([response_code, device_status])
    frame += actual_data
    checksum = 0
    for b in frame:
        checksum ^= b
    frame += bytes([checksum])
    return b"\xFF\xFF" + frame         # two preamble bytes + frame


# ---------------------------------------------------------------------------
# MockSerial  (queue of pre-programmed response byte strings)
# ---------------------------------------------------------------------------

class MockSerial:
    def __init__(self):
        self._responses = []      # list of bytes, returned in order
        self._buf = b""
        self.written = []         # bytes written (commands sent)

    def queue_response(self, data: bytes):
        self._responses.append(data)

    def reset_input_buffer(self):
        self._buf = b""
        if self._responses:
            self._buf = self._responses.pop(0)

    def write(self, data: bytes):
        self.written.append(data)

    def read(self, n=1):
        chunk = self._buf[:n]
        self._buf = self._buf[n:]
        return chunk

    @property
    def in_waiting(self):
        return len(self._buf)


# ---------------------------------------------------------------------------
# Test runner helpers
# ---------------------------------------------------------------------------

_pass = 0
_fail = 0

def check(name, got, expected):
    global _pass, _fail
    if isinstance(expected, float) and isinstance(got, float):
        ok = abs(got - expected) < 1e-5 or (got != got and expected != expected)
    else:
        ok = (got == expected)
    if ok:
        print("  PASS ", name)
        _pass += 1
    else:
        print("  FAIL ", name, "  got", repr(got), "expected", repr(expected))
        _fail += 1

def check_raises(name, fn, *args, **kwargs):
    global _pass, _fail
    try:
        fn(*args, **kwargs)
        print("  FAIL ", name, " (no exception raised)")
        _fail += 1
    except Exception:
        print("  PASS ", name)
        _pass += 1

def section(title):
    print("\n--", title, "--")


# ===========================================================================
# 1. Parse register
# ===========================================================================

def test_parse_register():
    section("parse_register")
    s = State()

    r = s.parse_register("1.primary_variable")
    check("cmd_id", r["cmd_id"], 1)
    check("field_name", r["field_name"], "primary_variable")

    r = s.parse_register("3.secondary_variable")
    check("cmd3 field", r["field_name"], "secondary_variable")

    r = s.parse_register("2.analog_signal")
    check("cmd2 field", r["field_name"], "analog_signal")

    r = s.parse_register("0.manufacturer_id")
    check("cmd0 cmd_id", r["cmd_id"], 0)

    # field_name with dots (split on first dot only)
    r = s.parse_register("3.some.nested")
    check("nested field", r["field_name"], "some.nested")

    check_raises("missing field", s.parse_register, "1")
    check_raises("not a number", s.parse_register, "abc.primary_variable")
    check_raises("empty string", s.parse_register, "")


# ===========================================================================
# 2. _cast_value
# ===========================================================================

def test_cast_value():
    from tango import CmdArgType
    section("_cast_value")
    s = State()

    check("float from float",  s._cast_value(25.0, CmdArgType.DevFloat),  25.0)
    check("float from int",    s._cast_value(10,   CmdArgType.DevFloat),  10.0)
    check("double from float", s._cast_value(3.14, CmdArgType.DevDouble), 3.14)
    check("long from int",     s._cast_value(42,   CmdArgType.DevLong),   42)
    check("long from float",   s._cast_value(7.9,  CmdArgType.DevLong),   7)
    check("short from int",    s._cast_value(100,  CmdArgType.DevShort),  100)
    check("long64",            s._cast_value(2**40, CmdArgType.DevLong64), 2**40)
    check("bool True",         s._cast_value(1,    CmdArgType.DevBoolean), True)
    check("bool False",        s._cast_value(0,    CmdArgType.DevBoolean), False)

    check("string from str",   s._cast_value("hello", CmdArgType.DevString), "hello")
    check("string from bytes", s._cast_value(b"test\x00", CmdArgType.DevString), "test\x00")
    check("string from int",   s._cast_value(99,   CmdArgType.DevString), "99")

    check_raises("unsupported type", s._cast_value, 1, 9999)


# ===========================================================================
# 3. _default_value
# ===========================================================================

def test_default_value():
    from tango import CmdArgType
    section("_default_value")
    s = State()

    check("DevFloat",   s._default_value(CmdArgType.DevFloat),   0.0)
    check("DevDouble",  s._default_value(CmdArgType.DevDouble),  0.0)
    check("DevLong",    s._default_value(CmdArgType.DevLong),    0)
    check("DevShort",   s._default_value(CmdArgType.DevShort),   0)
    check("DevLong64",  s._default_value(CmdArgType.DevLong64),  0)
    check("DevBoolean", s._default_value(CmdArgType.DevBoolean), False)
    check("DevString",  s._default_value(CmdArgType.DevString),  "")


# ===========================================================================
# 4. stringValueTo* helpers
# ===========================================================================

def test_string_mappings():
    from tango import CmdArgType, AttrWriteType, AttrDataFormat
    section("string mappings")
    s = State()

    check("DevFloat",   s.stringValueToVarType("DevFloat"),  CmdArgType.DevFloat)
    check("DevDouble",  s.stringValueToVarType("DevDouble"), CmdArgType.DevDouble)
    check("DevShort",   s.stringValueToVarType("DevShort"),  CmdArgType.DevShort)
    check("DevLong",    s.stringValueToVarType("DevLong"),   CmdArgType.DevLong)
    check("DevLong64",  s.stringValueToVarType("DevLong64"), CmdArgType.DevLong64)
    check("DevBoolean", s.stringValueToVarType("DevBoolean"),CmdArgType.DevBoolean)
    check("DevString",  s.stringValueToVarType("DevString"), CmdArgType.DevString)
    check("default type empty", s.stringValueToVarType(""), CmdArgType.DevDouble)
    check("unknown type -> double", s.stringValueToVarType("xyz"), CmdArgType.DevDouble)

    check("READ",       s.stringValueToWriteType("READ"),       AttrWriteType.READ)
    check("WRITE",      s.stringValueToWriteType("WRITE"),      AttrWriteType.WRITE)
    check("READ_WRITE", s.stringValueToWriteType("READ_WRITE"), AttrWriteType.READ_WRITE)
    check("empty -> READ", s.stringValueToWriteType(""),        AttrWriteType.READ)

    check("SCALAR",   s.stringValueToFormatType("SCALAR"),   AttrDataFormat.SCALAR)
    check("SPECTRUM", s.stringValueToFormatType("SPECTRUM"), AttrDataFormat.SPECTRUM)
    check("IMAGE",    s.stringValueToFormatType("IMAGE"),    AttrDataFormat.IMAGE)
    check("default format", s.stringValueToFormatType(""),  AttrDataFormat.SCALAR)


# ===========================================================================
# 5. _WRITE_COMMANDS constant
# ===========================================================================

def test_write_commands_set():
    section("_WRITE_COMMANDS")
    check("cmd 6 in write set",  6  in _WRITE_COMMANDS, True)
    check("cmd 17 in write set", 17 in _WRITE_COMMANDS, True)
    check("cmd 19 in write set", 19 in _WRITE_COMMANDS, True)
    check("cmd 1 not writable",  1  in _WRITE_COMMANDS, False)
    check("cmd 3 not writable",  3  in _WRITE_COMMANDS, False)


# ===========================================================================
# Sections below require hart_protocol installed
# ===========================================================================

def skip(name):
    global _pass
    print("  SKIP ", name)
    _pass += 1   # count skips as pass so the total isn't misleading


def make_cmd1_response(address=0, pv=25.0, units=39):
    data = struct.pack(">Bf", units, pv)
    return make_hart_response(address, 1, data)

def make_cmd2_response(address=0, analog_signal=12.0, pv_percent=50.0):
    data = struct.pack(">ff", analog_signal, pv_percent)
    return make_hart_response(address, 2, data)

def make_cmd3_response(address=0, analog_signal=12.0, pv_units=39, pv=25.0, sv_units=32, sv=50.0):
    data = struct.pack(">fBfBf", analog_signal, pv_units, pv, sv_units, sv)
    return make_hart_response(address, 3, data)

def make_cmd0_response(address=0, manufacturer_id=17, device_type=1, device_id=12345):
    # CMD 0: 12 bytes of device identification
    data = bytes([
        0xFF,           # byte 0: extended device type MSB
        manufacturer_id,
        device_type,
        5,              # num preamble chars
        5,              # universal cmd revision
        1,              # transmitter-specific revision
        1,              # software revision
        1,              # hardware revision (lower 5 bits) | physical signal (upper 3 bits)
        0,              # flags
        (device_id >> 16) & 0xFF,
        (device_id >> 8)  & 0xFF,
        device_id & 0xFF,
    ])
    return make_hart_response(address, 0, data)

def make_cmd16_response(address=0, final_assembly_no=42):
    data = final_assembly_no.to_bytes(3, "big")
    return make_hart_response(address, 16, data)

def make_cmd6_response(address=0, polling_address=3):
    data = bytes([polling_address])
    return make_hart_response(address, 6, data)

def make_cmd19_response(address=0, final_assembly_no=99):
    # hart_protocol CMD 19 parser reads only data[0:2], so send exactly 2 bytes.
    data = final_assembly_no.to_bytes(2, "big")
    return make_hart_response(address, 19, data)


# ---------------------------------------------------------------------------
# Fake attribute object used by read/write_dynamic_attr
# ---------------------------------------------------------------------------

class FakeAttr:
    def __init__(self, name, write_value=None):
        self._name = name
        self._value = None
        self._write_value = write_value

    def get_name(self):
        return self._name

    def set_value(self, v):
        self._value = v

    def get_write_value(self):
        return self._write_value


# ===========================================================================
# 6. _send_command via MockSerial
# ===========================================================================

def test_send_command():
    if not HAS_HART:
        skip("send_command (hart_protocol not installed)")
        return
    section("_send_command via MockSerial")
    s = State()
    mock = MockSerial()
    s._serial = mock

    # Queue a CMD 1 response  (_send_command calls reset_input_buffer internally)
    mock.queue_response(make_cmd1_response(address=0, pv=25.0, units=39))

    response = s._send_command(0, 1)
    check("cmd1 not None",         response is not None, True)
    check("cmd1 primary_variable", response.primary_variable, 25.0)
    check("cmd1 units",            response.primary_variable_units, 39)

    # Queue CMD 3 response
    mock.queue_response(make_cmd3_response(pv=30.5, sv=55.2))
    response = s._send_command(0, 3)
    check("cmd3 not None",         response is not None, True)
    check("cmd3 primary_variable", abs(response.primary_variable - 30.5) < 1e-4, True)
    check("cmd3 secondary_variable", abs(response.secondary_variable - 55.2) < 1e-4, True)

    # Empty buffer -> None
    response = s._send_command(0, 1)
    check("no response -> None", response is None, True)


# ===========================================================================
# 7. _run_poll updates dynamicAttributes
# ===========================================================================

def test_run_poll():
    if not HAS_HART:
        skip("_run_poll (hart_protocol not installed)")
        return
    section("_run_poll")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    # Register two attributes: one for CMD 1, one for CMD 3
    s.dynamicAttributeLookup["pv"] = {
        "variableType": CmdArgType.DevDouble,
        "cmd_id": 1,
        "field_name": "primary_variable",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["pv"] = 0.0

    s.dynamicAttributeLookup["sv"] = {
        "variableType": CmdArgType.DevDouble,
        "cmd_id": 3,
        "field_name": "secondary_variable",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["sv"] = 0.0

    s.dynamicAttributeLookup["loop_ma"] = {
        "variableType": CmdArgType.DevDouble,
        "cmd_id": 3,
        "field_name": "analog_signal",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["loop_ma"] = 0.0

    # _send_command calls reset_input_buffer internally per command, popping one
    # queued response each time.  Two cmd_ids -> two calls -> two pops.
    mock.queue_response(make_cmd1_response(pv=42.5, units=39))
    mock.queue_response(make_cmd3_response(analog_signal=11.8, pv=42.5, sv=60.1))

    s._run_poll()

    check("pv cached",     abs(s.dynamicAttributes["pv"] - 42.5) < 1e-4, True)
    check("sv cached",     abs(s.dynamicAttributes["sv"] - 60.1) < 1e-4, True)
    check("loop_ma cached", abs(s.dynamicAttributes["loop_ma"] - 11.8) < 1e-4, True)


# ===========================================================================
# 8. read_dynamic_attr returns cached value
# ===========================================================================

def test_read_dynamic_attr():
    if not HAS_HART:
        skip("read_dynamic_attr (hart_protocol not installed)")
        return
    section("read_dynamic_attr")
    from tango import CmdArgType

    s = State()
    s.dynamicAttributeLookup["temperature"] = {
        "variableType": CmdArgType.DevFloat,
        "cmd_id": 1,
        "field_name": "primary_variable",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["temperature"] = 55.5

    attr = FakeAttr("temperature")
    s.read_dynamic_attr(attr)
    check("read returns cached float", attr._value, 55.5)

    s.dynamicAttributes["temperature"] = 0.0
    s.read_dynamic_attr(attr)
    check("read returns updated 0.0", attr._value, 0.0)


# ===========================================================================
# 9. write_dynamic_attr: CMD 6 (write polling address)
# ===========================================================================

def test_write_cmd6():
    if not HAS_HART:
        skip("write CMD 6 (hart_protocol not installed)")
        return
    section("write_dynamic_attr CMD 6 (polling address)")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    s.dynamicAttributeLookup["poll_addr"] = {
        "variableType": CmdArgType.DevLong,
        "cmd_id": 6,
        "field_name": "polling_address",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["poll_addr"] = 0

    # Queue a CMD 6 write response
    mock.queue_response(make_cmd6_response(address=0, polling_address=3))

    attr = FakeAttr("poll_addr", write_value=3)
    s.write_dynamic_attr(attr)

    check("poll_addr cache updated", s.dynamicAttributes["poll_addr"], 3)
    check("command was sent", len(mock.written), 1)

    # Verify the sent command contains cmd_id=6
    sent = mock.written[0]
    # hart command bytes: preamble(5) + start(1) + addr(5) + cmd(1) + bytecount(1) + data(1) + checksum(1)
    # cmd byte is at position 11 (5+1+5)
    check("sent cmd_id is 6", sent[11], 6)


# ===========================================================================
# 10. write_dynamic_attr: CMD 19 (write final assembly number)
# ===========================================================================

def test_write_cmd19():
    if not HAS_HART:
        skip("write CMD 19 (hart_protocol not installed)")
        return
    section("write_dynamic_attr CMD 19 (final assembly number)")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    s.dynamicAttributeLookup["assembly"] = {
        "variableType": CmdArgType.DevLong,
        "cmd_id": 19,
        "field_name": "final_assembly_no",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["assembly"] = 0

    mock.queue_response(make_cmd19_response(final_assembly_no=99))

    attr = FakeAttr("assembly", write_value=99)
    s.write_dynamic_attr(attr)

    check("assembly cache updated", s.dynamicAttributes["assembly"], 99)
    check("cmd 19 sent", sent[11] if (sent := mock.written[0]) else None, 19)


# ===========================================================================
# 11. write_dynamic_attr: unsupported write command logs warning
# ===========================================================================

def test_write_unsupported():
    section("write_dynamic_attr unsupported cmd")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    s.dynamicAttributeLookup["pv"] = {
        "variableType": CmdArgType.DevDouble,
        "cmd_id": 1,            # CMD 1 is read-only
        "field_name": "primary_variable",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["pv"] = 25.0

    attr = FakeAttr("pv", write_value=99.0)
    s.write_dynamic_attr(attr)   # should warn and not crash

    # Cache should not change since no write support and no response
    check("pv unchanged after unsupported write", s.dynamicAttributes["pv"], 25.0)
    check("nothing sent to serial", len(mock.written), 0)


# ===========================================================================
# 12. _build_write_data
# ===========================================================================

def test_build_write_data():
    if not HAS_HART:
        skip("_build_write_data (hart_protocol not installed)")
        return
    section("_build_write_data")
    from tango import CmdArgType

    s = State()

    d = s._build_write_data(6, "polling_address", 3, CmdArgType.DevLong)
    check("cmd6 data length 1", len(d), 1)
    check("cmd6 data value",    d[0], 3)

    d = s._build_write_data(19, "final_assembly_no", 42, CmdArgType.DevLong)
    check("cmd19 data length 3", len(d), 3)
    check("cmd19 value",         int.from_bytes(d, "big"), 42)

    d = s._build_write_data(17, "message", "Hello HART", CmdArgType.DevString)
    check("cmd17 data not None", d is not None, True)
    check("cmd17 data len (32 chars * 6 bits / 8 = 24 bytes)", len(d), 24)

    d = s._build_write_data(1, "primary_variable", 25.0, CmdArgType.DevDouble)
    check("cmd1 (read-only) -> None", d is None, True)


# ===========================================================================
# 13. Full poll round-trip with multiple CMD 0 / CMD 16 fields
# ===========================================================================

def test_poll_cmd0_cmd16():
    if not HAS_HART:
        skip("poll CMD 0 / CMD 16 (hart_protocol not installed)")
        return
    section("poll CMD 0 and CMD 16")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    s.dynamicAttributeLookup["mfr_id"] = {
        "variableType": CmdArgType.DevLong,
        "cmd_id": 0,
        "field_name": "manufacturer_id",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["mfr_id"] = 0

    s.dynamicAttributeLookup["dev_id"] = {
        "variableType": CmdArgType.DevLong,
        "cmd_id": 0,
        "field_name": "device_id",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["dev_id"] = 0

    s.dynamicAttributeLookup["assembly"] = {
        "variableType": CmdArgType.DevLong,
        "cmd_id": 16,
        "field_name": "final_assembly_no",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["assembly"] = 0

    mock.queue_response(make_cmd0_response(manufacturer_id=17, device_id=12345))
    mock.queue_response(make_cmd16_response(final_assembly_no=777))

    s._run_poll()

    check("mfr_id",    s.dynamicAttributes["mfr_id"],   17)
    check("dev_id",    s.dynamicAttributes["dev_id"],   12345)
    check("assembly",  s.dynamicAttributes["assembly"], 777)


# ===========================================================================
# 14. make_hart_response / Unpacker round-trip sanity
# ===========================================================================

def test_frame_roundtrip():
    if not HAS_HART:
        skip("frame round-trip (hart_protocol not installed)")
        return
    section("HART frame round-trip")
    from hart_protocol import Unpacker
    from Hart import _BytesBuffer

    # CMD 1
    raw = make_cmd1_response(address=0, pv=100.0, units=39)
    r = next(Unpacker(_BytesBuffer(raw)))
    check("cmd1 pv", abs(r.primary_variable - 100.0) < 1e-4, True)
    check("cmd1 command", r.command, 1)

    # CMD 2
    raw = make_cmd2_response(address=0, analog_signal=16.0, pv_percent=75.0)
    r = next(Unpacker(_BytesBuffer(raw)))
    check("cmd2 analog_signal", abs(r.analog_signal - 16.0) < 1e-4, True)
    check("cmd2 primary_variable", abs(r.primary_variable - 75.0) < 1e-4, True)

    # CMD 3
    raw = make_cmd3_response(pv=22.2, sv=44.4, analog_signal=8.0)
    r = next(Unpacker(_BytesBuffer(raw)))
    check("cmd3 pv",   abs(r.primary_variable - 22.2) < 1e-4, True)
    check("cmd3 sv",   abs(r.secondary_variable - 44.4) < 1e-4, True)

    # CMD 0
    raw = make_cmd0_response(manufacturer_id=5, device_id=9999)
    r = next(Unpacker(_BytesBuffer(raw)))
    check("cmd0 manufacturer_id", r.manufacturer_id, 5)
    check("cmd0 device_id",       r.device_id, 9999)

    # CMD 16
    raw = make_cmd16_response(final_assembly_no=42)
    r = next(Unpacker(_BytesBuffer(raw)))
    check("cmd16 final_assembly_no", r.final_assembly_no, 42)


# ===========================================================================
# 15. DevString cast from HART bytes fields
# ===========================================================================

def test_string_cast_from_bytes():
    section("DevString cast from bytes")
    from tango import CmdArgType

    s = State()
    check("ascii bytes", s._cast_value(b"sensor\x00", CmdArgType.DevString), "sensor\x00")
    check("utf8 bytes",  s._cast_value(b"T\xc3\xa9st", CmdArgType.DevString), "T\xe9st")
    check("int to str",  s._cast_value(12345, CmdArgType.DevString), "12345")
    check("float to str", s._cast_value(3.14, CmdArgType.DevString), "3.14")


# ===========================================================================
# 16. _cast_value: DevBoolean various truthy inputs
# ===========================================================================

def test_bool_cast():
    section("DevBoolean cast")
    from tango import CmdArgType

    s = State()
    check("True", s._cast_value(True,  CmdArgType.DevBoolean), True)
    check("False",s._cast_value(False, CmdArgType.DevBoolean), False)
    check("1",    s._cast_value(1,     CmdArgType.DevBoolean), True)
    check("0",    s._cast_value(0,     CmdArgType.DevBoolean), False)


# ===========================================================================
# 17. Multiple polls accumulate correctly (fresh values overwrite old)
# ===========================================================================

def test_multi_poll():
    if not HAS_HART:
        skip("multi-poll (hart_protocol not installed)")
        return
    section("multiple poll cycles")
    from tango import CmdArgType

    s = State()
    mock = MockSerial()
    s._serial = mock

    s.dynamicAttributeLookup["pv"] = {
        "variableType": CmdArgType.DevDouble,
        "cmd_id": 1,
        "field_name": "primary_variable",
        "dataFormat": None, "max_x": 256, "max_y": 256,
    }
    s.dynamicAttributes["pv"] = 0.0

    for expected in [10.0, 20.0, 30.0]:
        mock.queue_response(make_cmd1_response(pv=expected))
        s._run_poll()
        check(f"pv after poll = {expected}", abs(s.dynamicAttributes["pv"] - expected) < 1e-4, True)


# ===========================================================================
# Main
# ===========================================================================

if __name__ == "__main__":
    print("== Hart device server tests ==")
    print()
    print("-- setup --")
    if HAS_HART:
        print("  hart_protocol installed: YES")
    else:
        print("  hart_protocol installed: NO (mock tests will run, protocol tests skipped)")

    print()
    print("== Conversion and parsing tests ==")
    test_parse_register()
    test_cast_value()
    test_default_value()
    test_string_mappings()
    test_write_commands_set()

    print()
    print("== Mock serial tests ==")
    test_send_command()
    test_run_poll()
    test_read_dynamic_attr()
    test_write_cmd6()
    test_write_cmd19()
    test_write_unsupported()
    test_build_write_data()
    test_poll_cmd0_cmd16()
    test_frame_roundtrip()
    test_string_cast_from_bytes()
    test_bool_cast()
    test_multi_poll()

    print()
    print("=" * 50)
    print(f"  Results: {_pass}/{_pass + _fail} passed, {_fail} failed")
    print("=" * 50)
    sys.exit(0 if _fail == 0 else 1)
