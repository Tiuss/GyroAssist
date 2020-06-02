# GyroAssist
Gyro Assist script for Space Engineers

IMPORTANT!
Grid need to have its pivot point properly aligned ("Up" of the pivot == "Up" of the ship), otherwise the script will not work properly.

Copy/Paste to programming block. It turns itself on by default.
WASD to move forward/backward/sideways
QE to rotate

Possible commands: (via PB "run" from toolbar)
"toggle" - toggles script on/off
"dampeners" - toggles if it should cancel velocity or let you drift (multiple modes). Disable front-facing thrusters manually if you want to achieve "cruise control" functionality
"linear" - toggles if it should assist in forward, backward, or both directions (use eg. if having strong thrust in one direction)
"lateral" - toggle lateral assistance

Output written to PB's screen by default. If you need to show it on any other screen, paste the "sample custom data.txt" into Programmable Block's Custom Data.
Remember that text surface numeration starts with "0" (if you have a block with multiple screen surfaces like the cockpit).

for more info & some background, visit those reddit posts:

https://www.reddit.com/r/spaceengineers/comments/dz806s/script_in_progress_helicopterquadcopter_steering/
https://www.reddit.com/r/spaceengineers/comments/fqtzeu/gyro_assist_script_progress_showcase_sample_builds/
https://www.reddit.com/r/spaceengineers/comments/fr6fdz/gyro_assist_script_more_tests_using_workshop_ship/
