"""
Acoustics for the scream demo.

  synth_scream   rising-pitch voiced scream with vowel formants, vibrato, breath noise
  synth_babble   speech-like distractor (low pitch, changing vowels)
  synth_fan      broadband fan/AC noise
  Room           6 x 5 x 3 m lab (pyroomacoustics image-source model), table in
                 the middle; renders what the rover's two INMP441 mics hear
  ScreamDetector laptop-side detector on a 16 kHz mono stream: pitch and
                 harmonicity in the scream range plus band energy over the floor

Real recordings can replace the synthetic scream: put WAV files in
simulation3d/screams/ and pass use_recordings=True.
"""
import glob
import os

import numpy as np
import pyroomacoustics as pra
import scipy.signal as ss

FS = 48000
ROOM = (6.0, 5.0, 3.0)
TABLE_ORIGIN = (2.1, 1.9)        # table corner (x, y) in the room; table top 0.75 m
HERE = os.path.dirname(os.path.abspath(__file__))


def _formant_filter(x, fs, formants):
    y = np.zeros_like(x)
    for f, bw, g in formants:
        b, a = ss.iirpeak(f, bw, fs)          # bw here is the resonance Q (5-8): ~100-200 Hz bandwidth
        y += g * ss.lfilter(b, a, x)
    return y


def synth_scream(dur, fs, rng):
    n = int(dur * fs)
    t = np.arange(n) / fs
    f_start, f_peak = rng.uniform(450, 700), rng.uniform(900, 1400)
    rise = rng.uniform(0.15, 0.35)
    f0 = np.where(t < rise, f_start + (f_peak - f_start) * t / rise, f_peak - (f_peak - f_start) * 0.4 * (t - rise) / dur)
    f0 *= 1 + 0.03 * np.sin(2 * np.pi * rng.uniform(5, 7) * t) + 0.01 * rng.standard_normal(n).cumsum() / np.sqrt(n)
    phase = 2 * np.pi * np.cumsum(f0) / fs
    src = sum((1.0 / k) * np.sin(k * phase + rng.uniform(0, 6.3)) for k in range(1, 12) if k * f0.max() < fs / 2 - 500)
    src = src + 0.15 * rng.standard_normal(n)                                  # breath / roughness
    vowel = [(rng.uniform(750, 950), 6.0, 1.0), (rng.uniform(1300, 1700), 7.0, 0.7), (rng.uniform(2500, 3000), 8.0, 0.4)]
    y = _formant_filter(src, fs, vowel)
    env = np.minimum(1, t / 0.05) * np.minimum(1, (dur - t) / 0.12)
    y *= env
    return (y / (np.abs(y).max() + 1e-9)).astype(np.float32)


def synth_babble(dur, fs, rng):
    n = int(dur * fs)
    t = np.arange(n) / fs
    f0 = rng.uniform(100, 220) * (1 + 0.15 * np.sin(2 * np.pi * rng.uniform(2, 4) * t))
    phase = 2 * np.pi * np.cumsum(f0) / fs
    src = sum((1.0 / k) * np.sin(k * phase) for k in range(1, 30))
    y = np.zeros(n)
    seg = int(0.18 * fs)
    for i in range(0, n, seg):                                                  # changing vowels = syllables
        v = [(rng.uniform(300, 800), 5, 1.0), (rng.uniform(900, 2200), 6, 0.6), (rng.uniform(2300, 3000), 8, 0.3)]
        y[i:i + seg] = _formant_filter(src[i:i + seg], fs, v) * np.hanning(len(src[i:i + seg]))
    return (y / (np.abs(y).max() + 1e-9)).astype(np.float32)


def synth_fan(dur, fs, rng):
    n = int(dur * fs)
    x = rng.standard_normal(n)
    b, a = ss.butter(2, [60, 6000], btype="band", fs=fs)
    y = ss.lfilter(b, a, x) + 0.3 * np.sin(2 * np.pi * rng.uniform(90, 130) * np.arange(n) / fs)
    return (y / (np.abs(y).max() + 1e-9)).astype(np.float32)


def load_recordings():
    import soundfile as sf
    clips = []
    for f in sorted(glob.glob(os.path.join(HERE, "screams", "*.wav"))):
        x, fs = sf.read(f, dtype="float32", always_2d=True)
        x = x.mean(axis=1)
        if fs != FS:
            x = ss.resample_poly(x, FS, fs).astype(np.float32)
        clips.append(x / (np.abs(x).max() + 1e-9))
    return clips


class Room:
    """Renders 2-mic captures for given rover poses. Positions are table-frame (m)."""

    def __init__(self, rng, absorption=None, fan=True, max_order=8):
        self.rng = rng
        self.absorption = absorption if absorption is not None else rng.uniform(0.15, 0.35)
        self.max_order = max_order
        self.fan = fan
        self.fan_pos = (0.4, 4.5, 2.6)                                       # ceiling corner unit

    def _to_room(self, x, y, z):
        return (TABLE_ORIGIN[0] + x, TABLE_ORIGIN[1] + y, z)

    def capture(self, src_xy, src_signal, rover_pose, scream_gain=1.0, fan_gain=0.04, dur=None):
        """Returns (left, right) at 48 kHz as heard by the rover's mics (upper deck, y = +/-50 mm)."""
        x, y, th = rover_pose
        c, s = np.cos(th), np.sin(th)
        mics = []
        for my in (0.05, -0.05):                                             # left, right
            mx = 0.0
            mics.append(self._to_room(x + c * mx - s * my, y + s * mx + c * my, 0.75 + 0.088))
        room = pra.ShoeBox(ROOM, fs=FS, materials=pra.Material(self.absorption), max_order=self.max_order,
                           air_absorption=True)
        n = len(src_signal) if dur is None else int(dur * FS)
        if src_signal is not None and scream_gain > 0:
            room.add_source(self._to_room(src_xy[0], src_xy[1], 0.80), signal=scream_gain * src_signal[:n])
        if self.fan and fan_gain > 0:
            room.add_source(self.fan_pos, signal=fan_gain * synth_fan(n / FS + 0.1, FS, self.rng))
        room.add_microphone_array(np.array(mics).T)
        room.simulate()
        sig = room.mic_array.signals[:, :n]
        sig = sig + 0.0015 * self.rng.standard_normal(sig.shape)            # INMP441 self-noise (~61 dB SNR)
        return sig[0].astype(np.float32), sig[1].astype(np.float32)


class ScreamDetector:
    """Laptop-side, on a 16 kHz mono stream (what the robot streams over Wi-Fi)."""

    def __init__(self, fs=16000):
        self.fs = fs
        self.floor = None

    def frame_features(self, x):
        x = x - x.mean()
        e = float(np.mean(x * x)) + 1e-12
        spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        f = np.fft.rfftfreq(len(x), 1 / self.fs)
        band = spec[(f > 500) & (f < 4000)].sum() / (spec.sum() + 1e-9)
        # pitch by subharmonic summation: candidate f0 scored by the energy at its
        # first 6 harmonics (geometric weights), so a 150 Hz voice whose 5th
        # harmonic sits on a formant is not mistaken for a 750 Hz scream
        mag = spec / (spec.max() + 1e-12)
        cands = np.arange(70.0, 1600.0, 5.0)
        df = f[1] - f[0]
        score = np.zeros(len(cands))
        for h in range(1, 7):
            idx = np.clip(np.round(cands * h / df).astype(int), 0, len(mag) - 1)
            score += (0.8 ** (h - 1)) * mag[idx] * (cands * h < self.fs / 2)
        f0 = float(cands[int(np.argmax(score))])
        ac = np.correlate(x, x, "full")[len(x) - 1:]
        ac /= ac[0] + 1e-12
        k = int(round(self.fs / f0))
        harm = float(ac[min(k, len(ac) - 1)])
        return e, band, harm, f0

    def detect(self, mono16k):
        """True if the clip contains a scream: 4+ frames (64 ms) whose fundamental is
        380-1600 Hz, strongly harmonic, energetic, and stable frame to frame."""
        n = int(0.064 * self.fs)
        hits, prev_f0 = 0, None
        for i in range(0, len(mono16k) - n, n // 2):
            e, band, harm, f0 = self.frame_features(mono16k[i:i + n])
            if self.floor is None or e < self.floor:
                self.floor = e
            else:
                self.floor *= 1.002
            snr_db = 10 * np.log10(e / (self.floor + 1e-12))
            ok = snr_db > 6 and band > 0.5 and harm > 0.5 and 380 < f0 < 1600
            stable = prev_f0 is not None and abs(f0 - prev_f0) / prev_f0 < 0.2
            if ok and stable:
                hits += 1
            prev_f0 = f0 if ok else None
        return hits >= 3


def to16k(x48):
    return ss.resample_poly(x48, 1, 3).astype(np.float32)
