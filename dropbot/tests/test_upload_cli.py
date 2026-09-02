"""Hardware-free tests for the ``dropbot-upload`` command line."""
import pytest

from dropbot.bin import upload as upload_cli


def test_defaults_to_newest_environment():
    args = upload_cli.parse_args([], environments=['teensy31', 'teensy40'])
    assert args.hardware_version == 'teensy40'
    assert args.port is None


def test_port_and_hardware_version_options():
    args = upload_cli.parse_args(['-p', 'COM5', '-b', 'teensy31'],
                                 environments=['teensy31', 'teensy40'])
    assert args.port == 'COM5'
    assert args.hardware_version == 'teensy31'


def test_unknown_hardware_version_is_rejected():
    with pytest.raises(SystemExit):
        upload_cli.parse_args(['-b', 'nope'], environments=['teensy31'])


def test_console_script_forwards_port(monkeypatch):
    calls = []
    monkeypatch.setattr(upload_cli, 'available_environments',
                        lambda: ['teensy31'])
    monkeypatch.setattr(upload_cli, 'upload_conda',
                        lambda *a, **kw: calls.append((a, kw)))

    upload_cli.upload(['--port', 'COM7'])

    assert calls == [(('dropbot',), {'env_name': 'teensy31',
                                     'extra_args': ['--upload-port', 'COM7']})]


def test_console_script_without_port_autodetects(monkeypatch):
    calls = []
    monkeypatch.setattr(upload_cli, 'available_environments',
                        lambda: ['teensy31'])
    monkeypatch.setattr(upload_cli, 'upload_conda',
                        lambda *a, **kw: calls.append((a, kw)))

    upload_cli.upload([])

    assert calls == [(('dropbot',), {'env_name': 'teensy31', 'extra_args': []})]
