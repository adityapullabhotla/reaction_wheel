Reaction Wheel Attitude Control Testbed

A ground testbed that reproduces satellite reaction-wheel attitude control on a free-spinning platform. A 3D-printed flywheel driven by a DC motor rotates a camera-carrying platform through conservation of angular momentum, letting the system reject disturbances and autonomously track a moving target.

What it does
Mode	Behavior
Disturbance rejection	Platform holds 0 RPM. When pushed by hand, the reaction wheel spins up to cancel the motion and brings the platform back to rest.
Autonomous tracking	Camera locates a green target, and the platform rotates to keep it centered in frame.
Saturation characterization	Motor duty cycle is ramped while platform RPM is logged, detecting the point where the wheel can no longer add momentum.
Results

Measured on the assembled testbed (see final report for full methodology).

Metric	Result
Wheel saturation time	16.92 s average over 3 trials, against a 10 s design requirement
Disturbance recovery	Platform returned to rest in 1–2 s after impulses of roughly −100 RPM and +40 RPM
Platform spin-down time	12.72 s (σ = 0.39 s), confirming low bearing friction
Tipping angle	42.1° (σ = 0.6°)
Flywheel mass range	195 g – 432 g (modular)
Total system mass	1.465 kg at maximum flywheel configuration

Disturbance rejection

Hand-applied impulses produce RPM spikes; the PID controller drives the wheel to cancel them and returns the platform to zero.

Cascaded control during tracking

Platform angular rate (inner loop) tracking the rate commanded by the vision loop (outer loop).

Saturation ramp

Duty cycle ramped 5% → 70% over 30 s with automatic saturation detection based on RPM gain within a rolling window.

Control architecture

The tracking mode uses two nested control loops — a common structure in spacecraft attitude control, where a slow attitude loop commands a fast rate loop.

Outer loop (vision, ~30 Hz): Frames are converted to HSV, thresholded for the target color, and filtered by contour area and extent to reject noise. The centroid's horizontal offset from frame center becomes the error signal, smoothed over a rolling window of frames. A PID controller converts that pixel error into a commanded platform rate.

Inner loop (rate, 50 Hz): Gyroscope Z-axis data from the BNO085 is converted from rad/s to RPM and compared against the commanded rate. A second PID controller outputs a signed value whose magnitude sets PWM duty cycle and whose sign selects motor direction on the H-bridge.

Loss-of-target behavior: When the target leaves the frame, the controller holds the last known direction for a short momentum window, then enters a slow search sweep that reverses direction periodically until the target is reacquired.

Hardware
Component	Detail
Compute	Raspberry Pi 5
IMU	BNO085 over I²C at 400 kHz
Camera	Raspberry Pi Camera via picamera2
Motor driver	H-bridge with independent PWM and enable lines per direction
Motor	Brushed DC gearmotor with quadrature encoder
Flywheel	240 mm diameter, 20 mm thick, spoked hoop geometry, 16 alternating M5/M6 clearance holes for adjustable rim mass
Structure	3D-printed base, motor support, and bearing block; REV Robotics UltraHex shaft
Power	Battery with inline fuse and master switch; 18 AWG power runs, 24 AWG signal runs

