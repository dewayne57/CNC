; Spindle remains off. Establish a clear working area and origin before running.
; Send one line at a time and wait for ok.
G21
G90
G17
G92 X0 Y0 Z0
G1 X10 Y0 F120
G3 X0 Y10 I-10 J0
G2 X10 Y0 R10
G3 I-10 J0
G2 Z2 I-10 J0
M114
G1 Z0
G1 X0 Y0
M18
