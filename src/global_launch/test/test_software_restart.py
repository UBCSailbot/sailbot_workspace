import importlib.util
import os
import socket
import sys
from pathlib import Path
from unittest.mock import Mock

import pytest
from launch import LaunchService
from launch.actions import ExecuteProcess, Shutdown, TimerAction
from launch.events.process import ProcessStarted


@pytest.mark.parametrize(
    ("packet", "node_stopped"),
    [(b"restart", True), (b"restart!", True), (None, True), (b"restart", False)],
)
def test_global_launch_restarts_only_on_request(packet, node_stopped, monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("SAILBOT_RESTART_SOCKET", "previous-launch")
    launch_env = os.environ.copy()
    argv = ["ros2", "launch", "global_launch", "main_launch.py", "mode:=production"]
    monkeypatch.setattr(sys, "argv", argv)

    path = Path(__file__).resolve().parents[1] / "main_launch.py"
    spec = importlib.util.spec_from_file_location("sailbot_global_launch", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    on_exit = Mock()
    monkeypatch.setattr(module, "atexit", on_exit)
    execve = Mock()
    monkeypatch.setattr(os, "execve", execve)

    def probe(context):
        completion = context.asyncio_loop.create_future()
        if node_stopped:
            completion.set_result(None)
        node = Mock(spec=ExecuteProcess)
        node.get_asyncio_future.return_value = completion
        context.emit_event_sync(
            ProcessStarted(action=node, name="probe", cmd=["probe"], cwd=None, env=None, pid=42)
        )
        if packet is not None:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
                sender.sendto(packet, context.environment["SAILBOT_RESTART_SOCKET"])
        return [TimerAction(period=0.1, actions=[Shutdown(reason="ordinary stop")])]

    # Exercise the global launch entry point with a probe in place of the boat's nodes.
    monkeypatch.setattr(module, "setup_launch", probe)
    service = LaunchService()
    service.include_launch_description(module.generate_launch_description())
    assert service.run() == 0
    assert not Path(os.environ["SAILBOT_RESTART_SOCKET"]).parent.exists()
    execve.assert_not_called()
    if packet == b"restart":
        on_exit.register.assert_called_once()
        relaunch = on_exit.register.call_args.args[0]
        relaunch()
    else:
        on_exit.register.assert_not_called()
    if packet == b"restart" and node_stopped:
        execve.assert_called_once_with(sys.executable, [sys.executable, *argv], launch_env)
    else:
        execve.assert_not_called()
