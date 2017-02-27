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
- [] Set up basic motor control
- [] Set up serial input
- [] Write functionalities into threaded code
- [] TBD