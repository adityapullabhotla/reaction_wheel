#!/usr/bin/env python3
"""
reaction_wheel.py
-----------------
Reaction wheel motor controller — Raspberry Pi 5
Motor  : REV HD Hex (REV-41-1291), 28 counts/rev, 1:1 gear ratio
Driver : BTS7960 H-Bridge

Pin mapping (BCM)
  GPIO 26 → RPWM  (forward PWM)
  GPIO 16 → LPWM  (reverse PWM)
  GPIO  6 → R_EN  (right enable)
  GPIO  5 → L_EN  (left enable)
  GPIO 17 → ENC_A (encoder channel A)
  GPIO 22 → ENC_B (encoder channel B)

Install deps:
  sudo apt install python3-gpiozero python3-lgpio
"""

import time
import threading
import signal
import sys
from gpiozero import PWMOutputDevice, OutputDevice, Button
from gpiozero.pins.lgpio import LGPIOFactory

# ── Constants ──────────────────────────────────────────────────────────────────
COUNTS_PER_REV   = 28       # REV HD Hex, quadrature, no gearbox
RPM_INTERVAL     = 0.5      # seconds between RPM prints
PWM_FREQ         = 10_000   # Hz

# ── Pin numbers (BCM) ──────────────────────────────────────────────────────────
RPWM_PIN, LPWM_PIN = 26, 16
R_EN_PIN, L_EN_PIN = 6, 5
ENC_A_PIN, ENC_B_PIN = 17, 22

# ── Setup ──────────────────────────────────────────────────────────────────────
factory = LGPIOFactory()

rpwm  = PWMOutputDevice(RPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
lpwm  = PWMOutputDevice(LPWM_PIN, frequency=PWM_FREQ, pin_factory=factory)
r_en  = OutputDevice(R_EN_PIN, initial_value=False, pin_factory=factory)
l_en  = OutputDevice(L_EN_PIN, initial_value=False, pin_factory=factory)

# Encoder: Button gives reliable rising/falling edge interrupts on Pi 5.
# pull_up=True uses the internal pull-up; set to False if you have external ones.
enc_a = Button(ENC_A_PIN, pull_up=True, pin_factory=factory)
enc_b = Button(ENC_B_PIN, pull_up=True, pin_factory=factory)

# ── Encoder counter ────────────────────────────────────────────────────────────
_count = 0
_lock  = threading.Lock()

def _on_encoder_edge():
    """Called on every rising/falling edge of Channel A. Channel B gives direction."""
    global _count
    direction = +1 if enc_b.is_pressed else -1
    with _lock:
        _count += direction

enc_a.when_pressed  = _on_encoder_edge   # rising  edge
enc_a.when_released = _on_encoder_edge   # falling edge

# ── Motor control ──────────────────────────────────────────────────────────────
duty_cycle = 0.0

def set_speed(duty: float):
    """Set motor speed. duty: -1.0 (full reverse) to +1.0 (full forward), 0 = brake."""
    global duty_cycle
    duty_cycle = max(-1.0, min(1.0, duty))
    if duty_cycle > 0:
        rpwm.value, lpwm.value = duty_cycle, 0.0
    elif duty_cycle < 0:
        rpwm.value, lpwm.value = 0.0, -duty_cycle
    else:                           # brake: ENs high, both PWM low
        rpwm.value, lpwm.value = 0.0, 0.0
    r_en.on(); l_en.on()
    print(f"[Motor] duty = {duty_cycle:+.2f}")

def coast():
    """Float motor terminals — slow spin-down."""
    rpwm.value = lpwm.value = 0.0
    r_en.off(); l_en.off()

# ── RPM reporter ───────────────────────────────────────────────────────────────
_running = True

def _rpm_thread():
    global _count
    prev_count, prev_time = 0, time.monotonic()
    while _running:
        time.sleep(RPM_INTERVAL)
        now = time.monotonic()
        with _lock:
            cur = _count
        delta_counts = cur - prev_count
        delta_min    = (now - prev_time) / 60.0
        rpm = (delta_counts / COUNTS_PER_REV) / delta_min
        direction = "FWD" if rpm >= 0 else "REV"
        print(f"[RPM]  {abs(rpm):7.1f}  {direction}  | duty={duty_cycle:+.2f}  total_counts={cur}")
        prev_count, prev_time = cur, now

threading.Thread(target=_rpm_thread, daemon=True).start()

# ── Shutdown ───────────────────────────────────────────────────────────────────
def _shutdown(sig=None, frame=None):
    global _running
    _running = False
    print("\n[Shutdown] Braking then coasting …")
    set_speed(0.0)
    time.sleep(0.3)
    coast()
    sys.exit(0)

signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)

# ── Main / demo loop ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Reaction Wheel Controller | 28 counts/rev | 1:1 ratio")
    print("Hand-spin the wheel at any time to read passive RPM.")
    print("Ctrl+C to exit.\n")

    # --- Replace the demo below with your own control logic ---
    print("Spinning up to 50% forward …")
    set_speed(0.50)
    time.sleep(3.0)

    print("Braking …")
    set_speed(0.0)
    time.sleep(2.0)

    print("Coasting — hand-spin to test passive RPM …")
    coast()

    while _running:
        # Your control logic here: set_speed(new_duty)
        time.sleep(0.1)