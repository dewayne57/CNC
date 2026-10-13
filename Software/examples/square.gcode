; Send each line individually and wait for ok. Establish origin manually first.
G21
G90
G92 X0 Y0 Z0
G1 X10 Y0 F120
G1 X10 Y10
G1 X0 Y10
G1 X0 Y0
G4 P500
M114
M18
