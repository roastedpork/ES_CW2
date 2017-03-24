# TEAM MAJULAH

Members:
- Benjamin Ng Zhi Long
- Bingjie Chan
- Jia Ming Ang

## How to use our controller 

![Instructions](images/screen_initialization.PNG)

- Rotation             : R(value)
- Velocity             : V(value)
- Rotation & Velocity  : R(value)V(value)
- Tune Playing         : T\[(note)(duration)\]{1,16}
- Set tempo (in BPM)   : M(value)

## Other things to note

Our motor does not have enough torque to overcome the stalling friction, even at 100% duty cycle.
Hence we needed to give it a slight push for every rotation operation.
We hope that you would prod the motor whenever it give a duty cycle value of 100 even though the motor has yet completed its set number of rotations.

We also did not manage to get the auto-tuner to work.