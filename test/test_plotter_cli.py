"""Exercise plotter export options through the real executable, including headless operation."""

import os
from pathlib import Path
import shutil
import subprocess
import time

import pytest

PROFILE = Path(__file__).resolve().parents[1] / 'config' / 'example_straight_forward.yaml'


@pytest.fixture
def plotter():
    """Use the current build under CTest, or the sourced installation for direct pytest runs."""
    executable = (
        os.environ.get('PLOTTER_EXECUTABLE') or shutil.which('ground_vehicle_motion_plotter')
    )
    assert executable is not None and Path(executable).is_file()
    return executable


def headless_environment():
    """Remove display variables so the executable must select its own offscreen backend."""
    environment = os.environ.copy()
    for name in ('DISPLAY', 'WAYLAND_DISPLAY', 'QT_QPA_PLATFORM'):
        environment.pop(name, None)
    return environment


@pytest.mark.parametrize('selection', ['velocity', 'trajectory', 'both'])
def test_exports_only_requested_figures_without_a_display(plotter, tmp_path, selection):
    """Export one or both PNG files with paths containing spaces and without opening windows."""
    velocity = tmp_path / 'velocity profile.png'
    trajectory = tmp_path / 'robot trajectory.png'
    options = ['--no-show']
    if selection in ('velocity', 'both'):
        options.extend(['--save-vel-profile', str(velocity)])
    if selection in ('trajectory', 'both'):
        options.extend(['--save-trajectory', str(trajectory)])
    result = subprocess.run(
        [plotter, *options, str(PROFILE)], env=headless_environment(),
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 0, result.stderr
    for path, requested in (
        (velocity, selection in ('velocity', 'both')),
        (trajectory, selection in ('trajectory', 'both')),
    ):
        assert path.exists() == requested
        if requested:
            assert path.read_bytes().startswith(b'\x89PNG\r\n\x1a\n')


@pytest.mark.parametrize(
    'options',
    [
        ['--no-show'],
        ['--save-vel-profile'],
        ['--save-trajectory', '--no-show'],
        ['--save-trajectory', ''],
        ['--unknown'],
        ['another.yaml'],
        ['--save-vel-profile', 'one.png', '--save-vel-profile', 'two.png'],
        ['--save-vel-profile', 'same.png', '--save-trajectory', 'same.png'],
    ],
)
def test_rejects_invalid_cli_usage_without_a_display(plotter, options):
    """Reject incomplete or conflicting options before Qt attempts to connect to a display."""
    result = subprocess.run(
        [plotter, str(PROFILE), *options], env=headless_environment(),
        capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 2
    assert 'Usage:' in result.stdout
    assert 'ground_vehicle_motion_plotter:' in result.stderr


def test_help_and_missing_profile_need_no_display(plotter):
    """Explain every export flag in help and reject a command without a YAML argument."""
    environment = headless_environment()
    help_result = subprocess.run(
        [plotter, '--help'], env=environment, capture_output=True, text=True, timeout=10,
    )
    assert help_result.returncode == 0
    for flag in ('--save-vel-profile', '--save-trajectory', '--no-show'):
        assert flag in help_result.stdout
    missing = subprocess.run(
        [plotter], env=environment, capture_output=True, text=True, timeout=10,
    )
    assert missing.returncode == 2


@pytest.mark.parametrize('output', ['missing/figure.png', 'figure.jpg'])
def test_reports_export_errors(plotter, tmp_path, output):
    """Report an unavailable parent directory or unsupported output format as an export error."""
    result = subprocess.run(
        [plotter, str(PROFILE), '--save-trajectory', str(tmp_path / output), '--no-show'],
        env=headless_environment(), capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 1
    assert 'ground_vehicle_motion_plotter:' in result.stderr


def test_invalid_profile_fails_before_creating_qt_application(plotter, tmp_path):
    """Reject malformed profile data before preparing any GUI or output file."""
    profile = tmp_path / 'invalid.yaml'
    profile.write_text('limits: {}\n', encoding='utf-8')
    output = tmp_path / 'trajectory.png'
    environment = headless_environment()
    environment['QT_QPA_PLATFORM'] = 'nonexistent-platform'
    result = subprocess.run(
        [plotter, str(profile), '--save-trajectory', str(output), '--no-show'],
        env=environment, capture_output=True, text=True, timeout=10,
    )
    assert result.returncode == 1
    assert 'ground_vehicle_motion_plotter:' in result.stderr
    assert not output.exists()


@pytest.mark.parametrize('save', [False, True])
def test_keeps_interactive_mode_when_no_show_is_absent(plotter, tmp_path, save):
    """Keep the desktop event loop active after optional PNG exports until the user closes it."""
    environment = headless_environment()
    environment['QT_QPA_PLATFORM'] = 'offscreen'
    velocity = tmp_path / 'velocities.png'
    trajectory = tmp_path / 'trajectory.png'
    command = [plotter, str(PROFILE)]
    if save:
        command.extend(['--save-vel-profile', str(velocity), '--save-trajectory', str(trajectory)])
    process = subprocess.Popen(
        command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        if save:
            deadline = time.monotonic() + 5.0
            while not (velocity.exists() and trajectory.exists()):
                assert process.poll() is None, 'Plotter exited before exporting both figures'
                assert time.monotonic() < deadline, 'Plotter did not export both figures'
                time.sleep(0.02)
        time.sleep(0.2)
        assert process.poll() is None, 'Interactive mode exited without a close request'
    finally:
        process.terminate()
        process.communicate(timeout=5)
