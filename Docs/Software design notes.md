# Leg Motion Generator
## Inputs
* Velocity
	* angle relative to "front"
	* stride length (meters)
	* stride period (seconds)
* body pose
	* height
	* roll, pitch and yaw
* Stride Phase (normalized time)
* Preferred gait (amble, trot, ...)
## Outputs
* foot position in (x, y, z)
* foot velocity in (x', y', z')

## Implementation
### Startup
* initialize Leg object for each leg (call constructor function)
	* phase set to 0.0 (the neutral position)
* initialize body object (call constructor function)
	* set to "flat" and normal height above ground
* Start servo loop (run at maybe 20 Hz or faster)
	* find the foot contact plygon. This could be a point, line, triangle or quadrilateral
	* translate body so that center of gravity (accountng for direction gravity vector) is inside the foot-contact polygon
	* rotate body angles as much as possible to desired pose.
	* Adjust stance width or length if required to meet current stability goal.

