"""
I2CDevice — smbus2 wrapper with per-call timeout.

smbus2 has no native timeout. A stuck I2C bus (loose pin, EM noise, contention
with another bus master) can hang `read_byte()` indefinitely and freeze the
calling node — watchdog then has to kill it. This wrapper offloads each
transaction to a dedicated worker thread and waits with a deadline. On hang
the executor is abandoned (the stuck thread keeps running but is no longer
referenced) and a fresh executor is created so subsequent calls don't queue
behind the dead one.

Usage::

    from pi_nodes.hardware.i2c_device import I2CDevice, I2CTimeout

    dev = I2CDevice(bus_num=1, address=0x68, timeout_s=0.1, name='mpu6050')
    try:
        raw = dev.read_i2c_block_data(0x3B, 14)
    except I2CTimeout:
        ...  # bus stuck — drop sample, schedule recovery
    except OSError:
        ...  # bus error (NACK, etc.)
"""

import logging
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError as _FutTimeout

try:
    import smbus2
    _HAS_SMBUS = True
except ImportError:
    smbus2 = None
    _HAS_SMBUS = False


class I2CTimeout(Exception):
    """An I2C transaction did not complete within the configured deadline."""


class I2CDevice:
    """Per-device wrapper around smbus2.SMBus with a hard timeout.

    Not thread-safe across multiple I2CDevice instances on the same bus —
    callers must serialize externally if they share a `smbus2.SMBus` handle.
    Within one I2CDevice instance, calls are serialized internally.
    """

    def __init__(self, bus_num: int = 1, address: int = 0,
                 timeout_s: float = 0.1, name: str = 'i2c'):
        if not _HAS_SMBUS:
            raise RuntimeError('smbus2 not available — install on Pi')
        self._bus = smbus2.SMBus(bus_num)
        self._addr = address
        self._timeout = float(timeout_s)
        self._name = name
        self._lock = threading.Lock()
        self._exec = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix=f'i2c_{name}')
        self._log = logging.getLogger(f'i2c.{name}')
        self.timeout_count = 0
        self.error_count = 0

    # ── Internals ──────────────────────────────────────────────
    def _call(self, fn, *args):
        with self._lock:
            executor = self._exec
            try:
                future = executor.submit(fn, *args)
            except RuntimeError:
                # Executor was shut down (e.g. previous timeout) — recreate.
                self._exec = ThreadPoolExecutor(
                    max_workers=1, thread_name_prefix=f'i2c_{self._name}')
                executor = self._exec
                future = executor.submit(fn, *args)
        try:
            return future.result(timeout=self._timeout)
        except _FutTimeout:
            self.timeout_count += 1
            # The submitted task may still be blocked on the bus. We can't
            # cancel a thread that's mid-syscall, but we can abandon the
            # executor so future calls don't queue behind it.
            with self._lock:
                try:
                    self._exec.shutdown(wait=False, cancel_futures=False)
                except Exception:
                    pass
                self._exec = ThreadPoolExecutor(
                    max_workers=1, thread_name_prefix=f'i2c_{self._name}')
            raise I2CTimeout(
                f'{self._name}: I2C op exceeded {self._timeout}s')
        except Exception:
            self.error_count += 1
            raise

    # ── Public ops ─────────────────────────────────────────────
    def read_byte(self):
        return self._call(self._bus.read_byte, self._addr)

    def write_byte(self, value: int):
        return self._call(self._bus.write_byte, self._addr, value)

    def read_byte_data(self, register: int):
        return self._call(self._bus.read_byte_data, self._addr, register)

    def write_byte_data(self, register: int, value: int):
        return self._call(self._bus.write_byte_data,
                          self._addr, register, value)

    def read_word_data(self, register: int):
        return self._call(self._bus.read_word_data, self._addr, register)

    def read_i2c_block_data(self, register: int, length: int):
        return self._call(self._bus.read_i2c_block_data,
                          self._addr, register, length)

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass
        with self._lock:
            try:
                self._exec.shutdown(wait=False, cancel_futures=False)
            except Exception:
                pass
