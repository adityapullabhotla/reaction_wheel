from gpiozero import RotaryEncoder
from signal import pause

encoder = RotaryEncoder(a = 17, b= 22, max_steps=0)
def moved():
    print("Ticks:", encoder.steps)

encoder.when_rotated = moved

pause()