#!/usr/bin/env python3
"""
Unit tests for OrientationPublisher (dof_publisher).

Usage:
    cd poc/esp32-dof
    mamba run -n esp32-dof python3 -m pytest -q monitor/test_esp32dof_publisher.py
"""

import io
import json
import math
import re
import socket
import sys
from pathlib import Path

import pytest

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "webots/controllers/esp32_dof_follower"))

import dof_monitor
from dof_monitor import run, main
from dof_fusion import ComplementaryFilter
from dof_frame import Frame, format_frame
from dof_publisher import OrientationPublisher
from dof_sources import mock_lines as real_mock_lines, mock_angles, mock_frame
from dof_serial import DofSerialError
from dof_webots_math import parse_orientation_msg, euler_to_axis_angle


def fast(n_s=1.0, drop_every=0):
    """Fast mock source (non-realtime)."""
    return list(real_mock_lines(duration_s=n_s, realtime=False, drop_every=drop_every))


def rows(text):
    """Extract data rows (lines starting with a digit after split)."""
    return [l for l in text.splitlines() if l.split() and l.split()[0].isdigit()]


@pytest.fixture
def recv_sock():
    """Fixture: bind a socket on 127.0.0.1 and return (sock, port)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    _, port = s.getsockname()
    yield s, port
    s.close()


def drain(sock):
    """Drain all pending datagrams from a socket (setblocking(False)) and return list of bytes."""
    sock.setblocking(False)
    msgs = []
    try:
        while True:
            data, _ = sock.recvfrom(2048)
            msgs.append(data)
    except BlockingIOError:
        pass
    return msgs


class Fake:
    """Fake publisher for testing run/main."""
    def __init__(self):
        self.calls = []

    def publish(self, seq, r, p, y):
        self.calls.append((seq, r, p, y))

    @property
    def sent(self):
        return len(self.calls)

    def close(self):
        pass


class SpyFilter(ComplementaryFilter):
    """ComplementaryFilter that records dt values."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dts = []

    def update(self, accel_g, gyro_dps, dt_s):
        self.dts.append(dt_s)
        return super().update(accel_g, gyro_dps, dt_s)


# === Publisher unit tests (tests 1-7) ===

class TestPublisherBasics:
    """Publisher unit tests."""

    def test_roundtrip(self, recv_sock):
        """Test: publish a datagram and receive it."""
        sock, port = recv_sock
        pub = OrientationPublisher(port=port)
        pub.publish(7, 0.1, -0.2, 0.3)
        sock.settimeout(2)
        data, _ = sock.recvfrom(2048)
        obj = json.loads(data)
        assert obj == {"seq": 7, "roll": 0.1, "pitch": -0.2, "yaw": 0.3}
        assert list(obj.keys()) == ["seq", "roll", "pitch", "yaw"]
        assert parse_orientation_msg(data) == (0.1, -0.2, 0.3)
        assert pub.sent == 1
        assert pub.errors == 0
        pub.close()

    def test_int_angles_sent_as_float(self, recv_sock):
        """Test: integer angles are sent as floats."""
        sock, port = recv_sock
        pub = OrientationPublisher(port=port)
        pub.publish(1, 0, 0, 0)
        sock.settimeout(2)
        data, _ = sock.recvfrom(2048)
        obj = json.loads(data)
        assert isinstance(obj["roll"], float)
        pub.close()

    def test_nothing_listening_does_not_raise(self):
        """Test: sendto to a closed port doesn't raise."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        _, port = s.getsockname()
        s.close()
        pub = OrientationPublisher(port=port)
        for _ in range(5):
            pub.publish(1, 0.0, 0.0, 0.0)
        assert pub.sent + pub.errors == 5
        pub.close()

    def test_refuses_non_finite_and_bad_types(self, recv_sock):
        """Test: non-finite and bad types are refused."""
        sock, port = recv_sock
        pub = OrientationPublisher(port=port)
        bad_calls = [
            (1, math.nan, 0, 0),
            (1, 0, math.inf, 0),
            (1, 0, 0, -math.inf),
            (True, 0, 0, 0),
            ("1", 0, 0, 0),
            (1, "0", 0, 0),
            (1, True, 0, 0),
        ]
        for call in bad_calls:
            pub.publish(*call)
        sock.settimeout(0.2)
        with pytest.raises(socket.timeout):
            sock.recvfrom(2048)
        assert pub.skipped == 7
        assert pub.sent == 0
        assert pub.errors == 0
        pub.close()

    def test_close_idempotent_and_publish_after_close_silent(self):
        """Test: close is idempotent and publish after close is silent."""
        pub = OrientationPublisher()
        pub.close()
        pub.close()
        pub.publish(1, 0, 0, 0)
        assert pub.sent == 0
        pub.close()

    def test_bad_ctor_args(self):
        """Test: bad constructor args raise ValueError."""
        bad_args = [
            {"port": 0},
            {"port": 70000},
            {"port": True},
            {"port": "5005"},
            {"port": -1},
            {"host": ""},
            {"host": 5},
        ]
        for kwargs in bad_args:
            with pytest.raises(ValueError):
                OrientationPublisher(**kwargs)

    def test_default_port_5005_and_host(self):
        """Test: default port and host."""
        pub = OrientationPublisher()
        assert pub._addr == ("127.0.0.1", 5005)
        pub.close()


# === run() integration tests (tests 8-16) ===

class TestRunIntegration:
    """Tests for run() with publisher and fuser."""

    def test_run_publishes_once_per_good_frame_and_tracks_mock(self):
        """Test: run publishes once per good frame with correct angles."""
        lines = fast(10.0)
        f = Fake()
        run(iter(lines), out=io.StringIO(), quiet_every=1, fuser=ComplementaryFilter(), publisher=f)
        assert len(f.calls) == 500
        assert [c[0] for c in f.calls] == list(range(500))
        mr, mp = mock_angles(499 / 50)
        assert abs(f.calls[-1][1] - mr) <= 0.05
        assert abs(f.calls[-1][2] - mp) <= 0.05
        assert abs(f.calls[-1][3]) < 1e-9
        assert abs(f.calls[0][1]) < 1e-9

    def test_run_tracking_max_error_with_drops(self):
        """Test: run with drops, verify drop count and angle tracking."""
        lines = fast(10.0, 10)
        f = Fake()
        stats = run(iter(lines), out=io.StringIO(), quiet_every=1, fuser=ComplementaryFilter(), publisher=f)
        assert len(f.calls) == 450
        for seq, r, p, y in f.calls:
            mr, mp = mock_angles(seq / 50)
            assert abs(r - mr) <= 0.05
            assert abs(p - mp) <= 0.05
        assert stats.dropped == 49

    def test_run_dt_from_real_t_us(self):
        """Test: dt values match expected intervals."""
        spy = SpyFilter()
        lines = fast(0.5, 10)
        run(iter(lines), out=io.StringIO(), fuser=spy)
        assert spy.dts[0] == 0.0
        assert len(spy.dts) == 23
        dts_approx_04 = sum(1 for dt in spy.dts[1:] if abs(dt - 0.04) < 0.001)
        dts_approx_02 = sum(1 for dt in spy.dts[1:] if abs(dt - 0.02) < 0.001)
        assert dts_approx_04 == 2
        assert dts_approx_02 == 20

    def test_run_out_of_order_skips_fusion_and_publish(self):
        """Test: out-of-order frames skip fusion and publish."""
        lines = fast(0.2)
        lines.insert(6, lines[3])
        spy = SpyFilter()
        f = Fake()
        out = io.StringIO()
        stats = run(iter(lines), out=out, quiet_every=1, fuser=spy, publisher=f)
        assert stats.out_of_order == 1
        assert stats.received == 11
        assert len(f.calls) == 10
        assert len(spy.dts) == 10
        assert [c[0] for c in f.calls] == list(range(10))
        r = rows(out.getvalue())
        assert len(r) == 11
        assert r[6].split()[0] == "3"
        assert r[6].split()[-3:] == r[5].split()[-3:]

    def test_run_reset_calls_fuser_reset_and_reinits(self):
        """Test: reset calls fuser.reset() and reinitializes from accel."""
        lines = [format_frame(mock_frame(k)).rstrip("\r\n") for k in [200, 201, 202, 203, 204, 0, 1, 2, 3, 4]]
        f = ComplementaryFilter()
        orig = f.reset
        n = []
        f.reset = lambda: (n.append(1), orig())[1]
        pub = Fake()
        stats = run(iter(lines), out=io.StringIO(), quiet_every=1, fuser=f, publisher=pub)
        assert stats.resets == 1
        assert n == [1]
        assert abs(pub.calls[5][1]) < 1e-9
        assert abs(pub.calls[5][3]) < 1e-9
        assert abs(pub.calls[9][1] - mock_angles(4 / 50)[0]) <= 0.05

    def test_run_fuse_table_has_13_columns(self):
        """Test: fused table has 13 columns."""
        out = io.StringIO()
        run(iter(fast()), out=out, quiet_every=1, fuser=ComplementaryFilter())
        r = rows(out.getvalue())
        assert len(r) == 50
        for row in r:
            assert len(row.split()) == 13
        assert len(out.getvalue().splitlines()[0].split()) == 13
        assert "published" not in out.getvalue()

    def test_run_without_fuser_unchanged_and_publisher_requires_fuser(self):
        """Test: without fuser, 10 columns; publisher requires fuser."""
        out = io.StringIO()
        run(iter(fast()[:3]), out=out, quiet_every=1)
        r = rows(out.getvalue())
        assert len(r[0].split()) == 10
        with pytest.raises(ValueError):
            run(iter([]), out=io.StringIO(), publisher=Fake())

    def test_run_summary_published_line(self):
        """Test: summary includes published count."""
        out = io.StringIO()
        f = Fake()
        run(iter(fast()), out=out, quiet_every=1, fuser=ComplementaryFilter(), publisher=f)
        text = out.getvalue()
        assert re.search(r"published\s*:\s*50\b", text)
        out2 = io.StringIO()
        class NoPub:
            def publish(self, *args): pass
            def close(self): pass
        run(iter(fast()), out=out2, quiet_every=1, fuser=ComplementaryFilter(), publisher=NoPub())
        assert "published" not in out2.getvalue()

    def test_run_nonfinite_frame_values_never_reach_publisher(self):
        """Test: non-finite frame values don't reach publisher."""
        accel_zero_frame = Frame(0, 0, (0, 0, 0), (0, 0, 0))
        lines = [format_frame(accel_zero_frame).rstrip("\r\n"), format_frame(mock_frame(1)).rstrip("\r\n")]
        f = Fake()
        run(iter(lines), out=io.StringIO(), quiet_every=1, fuser=ComplementaryFilter(), publisher=f)
        for _, r, p, y in f.calls:
            assert math.isfinite(r) and math.isfinite(p) and math.isfinite(y)


# === main() tests (tests 17-24) ===

class TestMainIntegration:
    """Tests for main() with publisher and fusion."""

    def test_main_fuse_no_publisher_constructed(self, capsys, monkeypatch):
        """Test: --fuse without --publish doesn't construct publisher."""
        monkeypatch.setattr(dof_monitor, "mock_lines", lambda **kw: iter(fast()))
        calls = []
        def fake_pub(**kw):
            calls.append(kw)
            pytest.fail("no publisher")
        monkeypatch.setattr(dof_monitor, "OrientationPublisher", fake_pub)
        rc = main(["--mock", "--fuse", "--duration", "1", "--quiet-every", "1"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert len(rows(out)) == 50
        assert len(rows(out)[0].split()) == 13
        assert not calls

    def test_main_publish_constructs_publisher_closes_and_implies_fuse(self, capsys, monkeypatch):
        """Test: --publish constructs publisher, closes it, and implies --fuse."""
        class FakePub:
            instances = []
            def __init__(self, **kw):
                self.kw = kw
                self.calls = []
                self.closed = False
                FakePub.instances.append(self)
            def publish(self, *args):
                self.calls.append(args)
            @property
            def sent(self):
                return len(self.calls)
            def close(self):
                self.closed = True
        FakePub.instances = []
        monkeypatch.setattr(dof_monitor, "OrientationPublisher", FakePub)
        monkeypatch.setattr(dof_monitor, "mock_lines", lambda **kw: iter(fast()))
        class Rec(ComplementaryFilter):
            alpha_list = []
            def __init__(self, *args, **kwargs):
                Rec.alpha_list.append(args[0] if args else kwargs.get("alpha", 0.98))
                super().__init__(*args, **kwargs)
        Rec.alpha_list = []
        monkeypatch.setattr(dof_monitor, "ComplementaryFilter", Rec)
        rc = main(["--mock", "--publish", "--udp-port", "6001", "--alpha", "0.5"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert FakePub.instances[0].kw == {"port": 6001}
        assert FakePub.instances[0].closed
        assert len(FakePub.instances[0].calls) == 50
        assert Rec.alpha_list == [0.5]
        assert "published" in out

    def test_main_publisher_closed_when_source_errors(self, capsys, monkeypatch):
        """Test: publisher is closed on source error."""
        class FakePub:
            instances = []
            def __init__(self, **kw):
                self.closed = False
                FakePub.instances.append(self)
            def publish(self, *args): pass
            def close(self):
                self.closed = True
        FakePub.instances = []
        def fake_source(**kw):
            def gen():
                raise DofSerialError("boom")
                yield
            return gen()
        monkeypatch.setattr(dof_monitor, "OrientationPublisher", FakePub)
        monkeypatch.setattr(dof_monitor, "mock_lines", fake_source)
        rc = main(["--mock", "--publish"])
        out, err = capsys.readouterr()
        assert rc == 2
        assert "dof_monitor: error:" in err
        assert FakePub.instances[0].closed

    def test_main_publisher_closed_on_keyboard_interrupt(self, capsys, monkeypatch):
        """Test: publisher is closed on KeyboardInterrupt."""
        class FakePub:
            instances = []
            def __init__(self, **kw):
                self.closed = False
                FakePub.instances.append(self)
            def publish(self, *args): pass
            def close(self):
                self.closed = True
        FakePub.instances = []
        def fake_source(**kw):
            def gen():
                yield from fast()[:3]
                raise KeyboardInterrupt
            return gen()
        monkeypatch.setattr(dof_monitor, "mock_lines", fake_source)
        monkeypatch.setattr(dof_monitor, "OrientationPublisher", FakePub)
        rc = main(["--mock", "--publish"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert FakePub.instances[0].closed

    def test_main_alpha_ignored_without_fuse(self, capsys, monkeypatch):
        """Test: --alpha is ignored without --fuse/--publish."""
        def fail(**kw):
            pytest.fail("no filter")
        monkeypatch.setattr(dof_monitor, "ComplementaryFilter", fail)
        monkeypatch.setattr(dof_monitor, "mock_lines", lambda **kw: iter(fast()))
        rc = main(["--mock", "--alpha", "0.5", "--quiet-every", "1"])
        out, err = capsys.readouterr()
        assert rc == 0
        assert len(rows(out)) == 50
        assert len(rows(out)[0].split()) == 10

    def test_e2e_fast_udp_contract(self, recv_sock, monkeypatch):
        """Test: full UDP contract with fast mock."""
        sock, port = recv_sock
        monkeypatch.setattr(dof_monitor, "mock_lines", lambda **kw: iter(fast(2.0)))
        rc = main(["--mock", "--publish", "--udp-port", str(port), "--duration", "1", "--quiet-every", "1000"])
        assert rc == 0
        msgs = drain(sock)
        assert len(msgs) == 100
        for m in msgs:
            assert parse_orientation_msg(m) is not None
        seqs = [json.loads(m)["seq"] for m in msgs]
        assert seqs == list(range(100))
        assert abs(float(json.loads(msgs[-1])["roll"]) - mock_angles(99 / 50)[0]) <= 0.05
        x, y, z, angle = euler_to_axis_angle(0.0, 0.0, 0.0)
        assert (x, y, z, angle) == (0.0, 0.0, 1.0, 0.0)
        for m in msgs:
            obj = json.loads(m)
            x, y, z, angle = euler_to_axis_angle(obj["roll"], obj["pitch"], obj["yaw"])
            assert math.isclose(x * x + y * y + z * z, 1.0, rel_tol=1e-9)
            assert 0 <= angle <= math.pi

    def test_e2e_realtime_udp(self, recv_sock, monkeypatch):
        """Test: realtime UDP (minimal; one run only)."""
        sock, port = recv_sock
        # Don't patch mock_lines; use default realtime behavior
        rc = main(["--mock", "--publish", "--udp-port", str(port), "--duration", "1", "--quiet-every", "1000"])
        assert rc == 0
        msgs = drain(sock)
        assert 45 <= len(msgs) <= 50
        for m in msgs:
            assert parse_orientation_msg(m) is not None

    def test_publish_without_listener_via_main(self, monkeypatch):
        """Test: publish without listener doesn't raise or Traceback."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        _, port = s.getsockname()
        s.close()
        monkeypatch.setattr(dof_monitor, "mock_lines", lambda **kw: iter(fast()))
        rc = main(["--mock", "--publish", "--udp-port", str(port)])
        assert rc == 0
