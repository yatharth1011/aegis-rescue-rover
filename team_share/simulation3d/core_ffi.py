"""ctypes bindings to the firmware core (built on first import)."""
import ctypes
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.join(HERE, "..", "firmware", "aegis_s3")
CORE = os.path.join(FW, "components", "aegis_core")
LIB = os.path.join(HERE, "build", "libaegis_core.dylib")
SRCS = ["core_shim.c"] + [os.path.join(CORE, f) for f in ("aegis_dsp.c", "aegis_control.c", "aegis_policy.c")]


def _build():
    os.makedirs(os.path.dirname(LIB), exist_ok=True)
    newest = max(os.path.getmtime(os.path.join(HERE, s) if not os.path.isabs(s) else s) for s in SRCS)
    if os.path.exists(LIB) and os.path.getmtime(LIB) > newest:
        return
    cmd = ["cc", "-O2", "-shared", "-fPIC", "-std=c11", "-I", os.path.join(CORE, "include"), "-I",
           os.path.join(HERE, "..")] + [os.path.join(HERE, s) if not os.path.isabs(s) else s for s in SRCS] + ["-o", LIB]
    subprocess.check_call(cmd)


_build()
_lib = ctypes.CDLL(LIB)
F = ctypes.POINTER(ctypes.c_float)
_lib.doa_new.restype = ctypes.c_void_p
_lib.doa_frame.argtypes = [ctypes.c_void_p, F, F, F]
_lib.doa_free.argtypes = [ctypes.c_void_p]
_lib.safety_filter.argtypes = [ctypes.c_float] * 3 + [ctypes.c_int] + [ctypes.c_float] * 3 + [F]
_lib.drive_new.restype = ctypes.c_void_p
_lib.drive_step.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, F, ctypes.c_float, ctypes.c_float, F]
_lib.drive_step.restype = ctypes.c_int
_lib.drive_free.argtypes = [ctypes.c_void_p]
_lib.kin_fwd.argtypes = [ctypes.c_float, ctypes.c_float, F]

FRAME = 1024
FS = 48000


def _p(a):
    return a.ctypes.data_as(F)


class DOA:
    def __init__(self):
        self.h = _lib.doa_new()
        self.out = np.zeros(5, np.float32)

    def frame(self, left, right):
        l = np.ascontiguousarray(left, np.float32).copy()
        r = np.ascontiguousarray(right, np.float32).copy()
        _lib.doa_frame(self.h, _p(l), _p(r), _p(self.out))
        return dict(bearing=float(self.out[0]), lag=float(self.out[1]), psr=float(self.out[2]),
                    band_db=float(self.out[3]), valid=bool(self.out[4] > 0.5))

    def __del__(self):
        try:
            _lib.doa_free(self.h)
        except Exception:
            pass


def safety_filter(v, w, d_front, valid, d_safe, alpha, v_rev):
    out = np.zeros(3, np.float32)
    _lib.safety_filter(v, w, d_front, int(valid), d_safe, alpha, v_rev, _p(out))
    return float(out[0]), float(out[1]), bool(out[2] > 0.5)


class Drive:
    def __init__(self):
        self.h = _lib.drive_new()
        self.duty = np.zeros(4, np.float32)

    def step(self, v, w, w_meas, vbat, dt):
        wm = np.ascontiguousarray(w_meas, np.float32)
        stalled = _lib.drive_step(self.h, v, w, _p(wm), vbat, dt, _p(self.duty))
        return self.duty.copy(), bool(stalled)

    def __del__(self):
        try:
            _lib.drive_free(self.h)
        except Exception:
            pass


def kin_forward(wl, wr):
    out = np.zeros(2, np.float32)
    _lib.kin_fwd(wl, wr, _p(out))
    return float(out[0]), float(out[1])
