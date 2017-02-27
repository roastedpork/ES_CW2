# Architecture

Three main areas to define interfaces:

* Main Motor Controller (Nested PID)
* Serial Communications with external
* Interrupt Routines for Encoder

# Pre-requisites

Using Keil uVision 5.0, drivers and install files to be transferred via USB thumbdrive.

# Delegation of work

* JM & Bng - Design a PID controller for position and velocity
* Bing - Handle serial input from computer (regex) and expand functionalities

# Timeline
- [ ] Set up basic motor control
- [ ] Set up serial input
- [ ] Write functionalities into threaded code
- [ ] TBD

# PID implementation

Pseudo-code for PID implenmentation

```cpp

#define CONTROL_PERIOD 0.01 // 100Hz control loop

int actual_state;
int target_state;

// Error terms for PID control
int error = 0;
int prev_error = 0;
float int_error = 0;
float curr_derr = 0;


// PID gain constants to be tuned
float kp;
float ki;
float kd;

while(1){
	// Update target_state based on serial input
	// Update actual_state based on interrupts
	// These values will be updated based on other threads
	// Just use a set value for now


	// Update error terms
	prev_error = error;
	error = target_state - actual_state;
	int_error += (error + prev_error)*0.5*CONTROL_PERIOD; // Trapezium approximation

	float d_error = (error - prev_error)/CONTROL_PERIOD;
	curr_derr = 0.2 * d_error + 0.8 * curr_derr; // optional, curr_derr is a weighted exponential average of previous d_error terms

	// Calculate next speed
	float next_speed = kp * error + kd * curr_derr + ki * int_error;

	// Set next speed
	motor.setSpeed(next_speed);

	// Control loop waiting period
	wait(CONTROL_PERIOD);
}
```