MVI V0R0, #4
BEQ BB3, V0R0, V0R0

:L_BB2
ADD VxR0, V0R0, V0R10
MUL VxR1, VxR0, V0R15
ADD VxR2, V0R31, VxR0
SR VxR4, VxR0, V0R25
SRI V0R4, VxR2, #4

:L_BB3
SLI V1R5, V0R0, #2
