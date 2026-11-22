# iq2raw.py in.npy out.iq16 [gain]  -> interleaved int16 I,Q (as the firmware gets it)
import sys, numpy as np
x = np.load(sys.argv[1]).astype(np.complex64)
g = float(sys.argv[3]) if len(sys.argv) > 3 else 32768.0      # watch.py stored int16/32768
o = np.empty(2 * len(x), np.int16)
o[0::2] = np.clip(np.round(x.real * g), -32767, 32767); o[1::2] = np.clip(np.round(x.imag * g), -32767, 32767)
o.tofile(sys.argv[2]); print('ok', len(x))
