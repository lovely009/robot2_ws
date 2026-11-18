"""USB configuration regression tests; the vendor/USB boundary is a local C fake."""
import ctypes as C
import errno
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

TOOLS = Path(__file__).parents[3] / 'micro_ros_agent/tools'
sys.path.insert(0, str(TOOLS))


@pytest.mark.parametrize('override', [None, 'environment', 'argument'])
def test_backend_library_selection(tmp_path, monkeypatch, override):
    import canfd_transports as transport
    monkeypatch.setenv('HOME', str(tmp_path))
    monkeypatch.delenv('ROBOT2_CONTROLCANFD_LIB', raising=False)
    monkeypatch.setenv('USB_DEVFS_PATH', 'test')
    directory = tmp_path / '.local/lib/robot2'
    directory.mkdir(parents=True)
    original = directory / 'libcontrolcanfd.so'
    original.touch()
    compatible = directory / 'libcontrolcanfd_robot2.so'
    compatible.touch()
    loaded = []

    class Library:
        def __getattr__(self, name):
            return lambda *args: None

    def load(path, mode):
        assert mode == C.RTLD_LOCAL
        loaded.append(Path(path))
        return Library()

    monkeypatch.setattr(transport.C, 'CDLL', load)
    monkeypatch.setattr(transport, 'usb_devices', lambda: [])
    if override == 'environment':
        monkeypatch.setenv('ROBOT2_CONTROLCANFD_LIB', str(original))
    sdk = transport.CtypesSDK(str(original) if override == 'argument' else None)
    try:
        assert loaded == [original if override else compatible]
    finally:
        sdk.close()


@pytest.fixture(scope='module')
def native_sdk(tmp_path_factory):
    compiler = shutil.which('cc')
    if compiler is None:
        pytest.skip('native compatibility tests require a C compiler')
    directory = tmp_path_factory.mktemp('usbcanfd sdk')
    source = directory / 'vendor.c'
    source.write_text(r'''
#include <errno.h>
static int active, control_result, set_result, set_calls, get_calls;
void fake_reset(int config, int result, int set_status) {
    active = config; control_result = result; set_result = set_status;
    set_calls = get_calls = 0;
}
int fake_set_calls(void) { return set_calls; }
int fake_get_calls(void) { return get_calls; }
int usb_control_msg(void *dev, int type, int request, int value, int index,
                    char *data, int size, int timeout) {
    get_calls++;
    if (dev != (void *)0x1234 || type != 0x80 || request != 8 ||
        value != 0 || index != 0 || size != 1 || timeout <= 0)
        return -EINVAL;
    *data = (char)active;
    return control_result;
}
int usb_set_configuration(void *dev, int config) {
    (void)dev; (void)config;
    set_calls++;
    return set_result;
}
int fake_sdk_open(int config) {
    return usb_set_configuration((void *)0x1234, config);
}
''')
    original = directory / 'libcontrolcanfd.so'
    subprocess.run([compiler, '-shared', '-fPIC', str(source), '-o', str(original)],
                   check=True, capture_output=True)
    subprocess.run([sys.executable, str(TOOLS / 'build_usbcanfd_compat.py'),
                    str(original)], check=True, capture_output=True)
    compatible = directory / 'libcontrolcanfd_robot2.so'
    sdk = C.CDLL(str(compatible), mode=C.RTLD_LOCAL)
    sdk.fake_reset.argtypes = [C.c_int, C.c_int, C.c_int]
    sdk.fake_sdk_open.argtypes = [C.c_int]
    return sdk, compiler, source, directory


def test_repeated_configuration_does_not_reset_usb_endpoints(native_sdk):
    sdk, *_ = native_sdk
    sdk.fake_reset(1, 1, -errno.ETIMEDOUT)
    assert sdk.fake_sdk_open(1) == 0
    assert sdk.fake_set_calls() == 0
    assert sdk.fake_get_calls() == 1


@pytest.mark.parametrize('active,requested', [(0, 1), (1, 2), (1, 0)])
def test_actual_configuration_changes_reach_vendor(native_sdk, active, requested):
    sdk, *_ = native_sdk
    sdk.fake_reset(active, 1, -errno.EBUSY)
    assert sdk.fake_sdk_open(requested) == -errno.EBUSY
    assert sdk.fake_set_calls() == 1


@pytest.mark.parametrize('result,expected', [
    (-errno.ETIMEDOUT, -errno.ETIMEDOUT),
    (-errno.EACCES, -errno.EACCES),
    (0, -errno.EPROTO),
])
def test_failed_configuration_read_never_resets_device(native_sdk, result, expected):
    sdk, *_ = native_sdk
    sdk.fake_reset(1, result, 0)
    assert sdk.fake_sdk_open(1) == expected
    assert sdk.fake_set_calls() == 0


def test_compatibility_is_not_injected_into_other_libraries(native_sdk):
    _, compiler, source, directory = native_sdk
    unrelated = directory / 'unrelated.so'
    subprocess.run([compiler, '-shared', '-fPIC', str(source), '-o', str(unrelated)],
                   check=True, capture_output=True)
    other = C.CDLL(str(unrelated), mode=C.RTLD_LOCAL)
    other.fake_reset(1, 1, -errno.EBUSY)
    assert other.fake_sdk_open(1) == -errno.EBUSY
    assert other.fake_set_calls() == 1
    assert other.fake_get_calls() == 0
