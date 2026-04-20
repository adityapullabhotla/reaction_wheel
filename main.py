"""
motor_controller.py  —  Raspberry Pi 5 / BTS7960 / REV HD Hex Motor
=====================================================================

HARDWARE (from schematic)
--------------------------
  GPIO_5  -> RPWM   Forward PWM  (software PWM via lgpio)
  GPIO_6  -> LPWM   Reverse PWM  (software PWM via lgpio)
  GPIO_16 -> R_EN   Right half-bridge enable
  GPIO_26 -> L_EN   Left half-bridge enable
  GPIO_17 -> Encoder Channel A  (interrupt-driven quadrature decoder)
  GPIO_22 -> Encoder Channel B

MOTOR SPECS  (REV HD Hex, no gearbox — 1:1)
--------------------------------------------
  Free speed:           6000 RPM  @ 12 V (duty = 1.0)
  Encoder resolution:   28 counts / revolution (at motor shaft)
  Voltage:              12 V DC
  Stall current:        8.5 A

WHY lgpio (not RPi.GPIO)
------------------------
  The Raspberry Pi 5 uses the RP1 I/O controller chip. RPi.GPIO does NOT
  work on Pi 5. lgpio is the officially recommended replacement and ships
  with Raspberry Pi OS Bookworm. GPIO 5 and 6 do not have hardware PWM
  capability on any Pi, so we use lgpio software PWM (tx_pwm), which is
  accurate enough at 1 kHz for brushed DC motor control.

INSTALLATION
------------
  sudo apt install python3-lgpio          # already on Bookworm
  # or:  pip install lgpio --break-system-packages

USAGE
-----
  python motor_controller.py

  Prompt commands:
    <duty 0.0-1.0> <cw|ccw>    e.g.  0.5 cw   or   0.3 ccw
    stop                        coast (EN pins LOW, terminals float)
    brake                       active brake (EN HIGH, PWM = 0)
    quit / exit

TELEMETRY  (printed every 0.1 s)
---------------------------------
  [timestamp]  RPM: +XXXX.X  |  Enc dir: CW/CCW/STOPPED  |  Counts: XXXXXX
  Troubleshooting notes appended inline when anomalies are detected.
"""

import lgpio
import time
import threading
import sys

# ---------------------------------------------------------------------------
# Pin assignments  (BCM / GPIO numbering)
# ---------------------------------------------------------------------------
PIN_RPWM  = 5     # Forward PWM  -> RPWM on H-Bridge
PIN_LPWM  = 6     # Reverse PWM  -> LPWM on H-Bridge
PIN_R_EN  = 16    # Right half-bridge enable
PIN_L_EN  = 26    # Left half-bridge enable
PIN_ENC_A = 17    # Encoder quadrature channel A  (interrupt source)
PIN_ENC_B = 22    # Encoder quadrature channel B  (sampled in ISR)

# ---------------------------------------------------------------------------
# Motor / encoder constants
# ---------------------------------------------------------------------------
MAX_RPM          = 6000    # HD Hex fr ee-speed limit (no gearbox, 12 V)
COUNTS_PER_REV   = 28      # Encoder ticks per motor shaft revolution
PWM_FREQ_HZ      = 1000    # Software PWM carrier frequency (Hz)
MIN_DUTY_CYCLE   = 0.05    # Below this the motor will not reliably spin
PRINT_INTERVAL_S = 0.1     # Telemetry print rate (seconds)

# Pi 5 exposes GPIO via /dev/gpiochip4  (RP1 chip)
# Earlier Pi models use gpiochip0 — change this if running on Pi 4 or earlier
GPIO_CHIP = 4

# ---------------------------------------------------------------------------
# Shared encoder state  (written in ISR callback, read in telemetry thread)
# ---------------------------------------------------------------------------
_enc_count = 0
_enc_lock  = threading.Lock()


def _encoder_callback(chip, gpio, level, tick):
    """
    lgpio alert callback — fires on both edges of Channel A.
    Samples Channel B to decode direction:
      A rising  + B HIGH  ->  forward  (+1)
      A rising  + B LOW   ->  reverse  (-1)
      A falling + B LOW   ->  forward  (+1)
      A falling + B HIGH  ->  reverse  (-1)
    """
    global _enc_count
    b = lgpio.gpio_read(chip, PIN_ENC_B)
    if level == lgpio.RISING_EDGE:
        delta = 1 if b else -1
    else:
        delta = -1 if b else 1
    with _enc_lock:
        _enc_count += delta


# ---------------------------------------------------------------------------
# Setup / teardown
# ---------------------------------------------------------------------------
def setup():
    """Open GPIO chip, configure all pins, start software PWM at 0 %."""
    h = lgpio.gpiochip_open(GPIO_CHIP)

    # Claim output pins (initial state LOW)
    for pin in (PIN_RPWM, PIN_LPWM, PIN_R_EN, PIN_L_EN):
        lgpio.gpio_claim_output(h, pin, 0)

    # Claim encoder input pins with internal pull-ups
    lgpio.gpio_claim_input(h, PIN_ENC_A, lgpio.SET_PULL_UP)
    lgpio.gpio_claim_input(h, PIN_ENC_B, lgpio.SET_PULL_UP)

    # Start software PWM on both PWM pins at 0 % duty cycle
    lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, 0)
    lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, 0)

    # Register quadrature decoder on Channel A (both edges)
    lgpio.gpio_set_alerts_func(h, PIN_ENC_A, _encoder_callback)
    lgpio.gpio_set_edge_func(h, PIN_ENC_A, lgpio.BOTH_EDGES)

    return h


def teardown(h):
    """Coast to stop, halt PWM, release all GPIO."""
    _coast(h)
    lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, 0)
    lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, 0)
    lgpio.gpiochip_close(h)
    print("[INFO] GPIO closed. Goodbye.")


# ---------------------------------------------------------------------------
# Motor commands
# ---------------------------------------------------------------------------
def _coast(h):
    """Both EN LOW -> motor terminals float -> slow friction stop."""
    lgpio.gpio_write(h, PIN_R_EN, 0)
    lgpio.gpio_write(h, PIN_L_EN, 0)
    lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, 0)
    lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, 0)


def _brake(h):
    """
    Both EN HIGH, both PWM LOW.
    Shorts M+ and M- to GND -> back-EMF drives braking current -> fast stop.
    See BTS7960 datasheet section 3 (Coast vs. Brake).
    """
    lgpio.gpio_write(h, PIN_R_EN, 1)
    lgpio.gpio_write(h, PIN_L_EN, 1)
    lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, 0)
    lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, 0)


def set_motor(h, duty_cycle, direction):
    """
    Drive the motor.

    Parameters
    ----------
    h : int
        lgpio chip handle.
    duty_cycle : float
        0.0 (stopped) to 1.0 (full speed).
        Values outside [0, 1] are clamped with a warning.
    direction : str
        'cw' | 'ccw' (case-insensitive; 'clockwise' / 'counterclockwise' ok).

    Returns
    -------
    bool
        True if command was accepted and applied.
        False if command was rejected (motor state left unchanged or coasted).
    """
    # --- Validate direction before touching any hardware ---
    dir_norm = direction.strip().lower().replace('-', '').replace(' ', '')
    if dir_norm in ('cw', 'clockwise'):
        forward = True
    elif dir_norm in ('ccw', 'counterclockwise'):
        forward = False
    else:
        print(f"[ERROR] Unknown direction '{direction}'. Use 'cw' or 'ccw'.")
        return False

    # --- Clamp out-of-range duty cycle ---
    if not (0.0 <= duty_cycle <= 1.0):
        clamped = max(0.0, min(1.0, duty_cycle))
        print(
            f"[WARN] Duty cycle {duty_cycle:.3f} is outside [0.0, 1.0] — "
            f"clamped to {clamped:.3f}."
        )
        duty_cycle = clamped

    # --- Zero duty -> coast ---
    if duty_cycle == 0.0:
        _coast(h)
        return True

    # --- Below minimum threshold -> reject ---
    if duty_cycle < MIN_DUTY_CYCLE:
        print(
            f"[WARN] Duty cycle {duty_cycle:.3f} is below the minimum "
            f"threshold ({MIN_DUTY_CYCLE}).  The motor will not spin reliably.\n"
            f"       Use 0.0 to coast, or >= {MIN_DUTY_CYCLE} to drive."
        )
        _coast(h)
        return False

    # --- Advisory warning near free-speed limit ---
    expected_rpm = duty_cycle * MAX_RPM
    if expected_rpm > MAX_RPM * 0.90:
        print(
            f"[WARN] Duty {duty_cycle:.2f} -> ~{expected_rpm:.0f} RPM is above 90 % "
            f"of free speed ({MAX_RPM} RPM).  Actual RPM will be lower under load."
        )

    # --- EN pins must be HIGH before asserting PWM (datasheet section 3) ---
    lgpio.gpio_write(h, PIN_R_EN, 1)
    lgpio.gpio_write(h, PIN_L_EN, 1)

    pct = duty_cycle * 100.0   # lgpio tx_pwm takes 0–100 %

    if forward:
        lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, pct)
        lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, 0)
    else:
        lgpio.tx_pwm(h, PIN_RPWM, PWM_FREQ_HZ, 0)
        lgpio.tx_pwm(h, PIN_LPWM, PWM_FREQ_HZ, pct)

    return True


# ---------------------------------------------------------------------------
# Telemetry thread
# ---------------------------------------------------------------------------
def telemetry_thread(stop_event):
    """
    Every PRINT_INTERVAL_S seconds:
      - Reads encoder count delta
      - Computes RPM (counts/s / counts_per_rev * 60)
      - Infers direction from sign of delta
      - Prints a status line with optional troubleshooting notes
    """
    global _enc_count
    prev_count = 0
    prev_time  = time.monotonic()

    while not stop_event.is_set():
        time.sleep(PRINT_INTERVAL_S)
        now = time.monotonic()
        dt  = now - prev_time

        with _enc_lock:
            current_count = _enc_count

        delta      = current_count - prev_count
        prev_count = current_count
        prev_time  = now

        if dt > 0:
            cps = delta / dt
            rpm = (cps / COUNTS_PER_REV) * 60.0
        else:
            rpm = 0.0

        # Direction from encoder sign
        if abs(rpm) < 5.0:
            enc_dir = "STOPPED"
        elif rpm > 0:
            enc_dir = "CW     "
        else:
            enc_dir = "CCW    "

        # Inline troubleshooting notes
        notes = []
        if abs(rpm) > MAX_RPM * 1.05:
            notes.append(
                "RPM exceeds free-speed limit — verify COUNTS_PER_REV "
                "or check for gearbox multiplier"
            )
        elif abs(rpm) > MAX_RPM * 0.90:
            notes.append("approaching max RPM")
        if 0 < abs(rpm) < 30:
            notes.append(
                "very low RPM — possible stall, or duty cycle below threshold"
            )

        note_str = "  <- " + " | ".join(notes) if notes else ""

        print(
            f"[{now:8.2f}s]  RPM: {rpm:+7.1f}  |  "
            f"Enc dir: {enc_dir}  |  "
            f"Counts: {current_count:7d}{note_str}"
        )


# ---------------------------------------------------------------------------
# CLI parser
# ---------------------------------------------------------------------------
HELP_TEXT = """\
Commands:
  <duty 0.0-1.0> <cw|ccw>  — drive motor  (e.g.  0.5 cw   0.3 ccw)
  stop                      — coast to a stop (slow, friction only)
  brake                     — active brake   (fast, shorts motor terminals)
  help                      — show this message
  quit / exit               — clean shutdown
"""


def parse_command(line):
    """
    Parse one line of user input.

    Returns
    -------
    (cmd, duty_cycle, direction)
      cmd is 'drive' | 'stop' | 'brake' | 'quit' | 'help'
      duty_cycle and direction are None for non-drive commands.

    Raises ValueError with a readable message on bad input.
    """
    parts = line.strip().lower().split()
    if not parts:
        raise ValueError("Empty input — type 'help' for usage.")

    keyword = parts[0]

    if keyword in ('stop', 'brake', 'help'):
        return keyword, None, None
    if keyword in ('quit', 'exit'):
        return 'quit', None, None

    # Otherwise expect: <float> <direction>
    if len(parts) < 2:
        raise ValueError(
            "Drive command requires two arguments.\n"
            "  Example:  0.5 cw    or    0.3 ccw"
        )

    try:
        dc = float(parts[0])
    except ValueError:
        raise ValueError(
            f"'{parts[0]}' is not a valid duty cycle. "
            "Use a number between 0.0 and 1.0."
        )

    return 'drive', dc, parts[1]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 64)
    print("  BTS7960 Motor Controller  —  REV HD Hex Motor  —  RPi 5")
    print("=" * 64)
    print(HELP_TEXT)

    try:
        h = setup()
    except Exception as exc:
        print(
            f"[ERROR] Could not open GPIO chip {GPIO_CHIP}: {exc}\n\n"
            f"  Troubleshooting:\n"
            f"    1. Confirm you are on a Raspberry Pi 5 (chip index = 4).\n"
            f"       For Pi 4 or earlier, change GPIO_CHIP = 0 in this script.\n"
            f"    2. Install lgpio:  sudo apt install python3-lgpio\n"
            f"    3. Add yourself to the gpio group:\n"
            f"         sudo usermod -aG gpio $USER   (then log out and back in)\n"
            f"    4. Or run with:  sudo python motor_controller.py\n"
        )
        sys.exit(1)

    stop_event = threading.Event()
    telem = threading.Thread(
        target=telemetry_thread, args=(stop_event,), daemon=True
    )
    telem.start()

    print("Motor ready. Telemetry printing every 0.1 s...\n")

    try:
        while True:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            try:
                cmd, dc, direction = parse_command(line)
            except ValueError as exc:
                print(f"[ERROR] {exc}")
                continue

            if cmd == 'quit':
                break

            elif cmd == 'help':
                print(HELP_TEXT)

            elif cmd == 'stop':
                _coast(h)
                print("[INFO] Coasting to stop.")

            elif cmd == 'brake':
                _brake(h)
                print("[INFO] Active brake engaged.")

            elif cmd == 'drive':
                ok = set_motor(h, dc, direction)
                if ok and dc >= MIN_DUTY_CYCLE:
                    print(
                        f"[OK]  Duty: {dc:.2f}  |  Dir: {direction.upper()}  "
                        f"|  Expected no-load RPM: ~{dc * MAX_RPM:.0f}"
                    )

    finally:
        stop_event.set()
        teardown(h)


if __name__ == "__main__":
    main()