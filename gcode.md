| B/F |Comm | Arguments | Description |
| - | --- | --------- | ----------- |
| F | G0 | [X<VAL>] [Y<VAL>] [Z<VAL>] [F<VAL>] | synchronized movement |
| F | G1 | [X<VAL>] [Y<VAL>] [Z<VAL>] [F<VAL>] [E<VAL>] | synchronized movement |
| F | G28 | [X] [Y] [Z] [F] [E] | home |
| F | G92 | [X<VAL>] [Y<VAL>] [Z<VAL>] [F<VAL>] [E<VAL>] | define position |
| F | M20 | none | list files in root folder |
| F | M21 | none | init sd card |
| F | M23 | <file_name> | open file to read |
| F | M25 | none | pause sd printing |
| F | M26 | S<position> | set sd file pos |
| F | M28 | A<file write start position>  D<file write end position> | get transfer size and begin if valid |
| F | M30 | <file_name> | open file to write |
| F | M31 | A<estimated time> L<number of lines> | variables of standalone |
| F | M32 | none | print variables of standalone |
| F | M33 | none | start print |
| F | M104 | S<VAL> | define temperature |
| F | M105 | none | read temperature | 
| F | M106 | none | turn the blower on |
| F | M107 | none | turn the blower off |
| F | M109 | S<VAL> | define temperature and block |
| F | M112 | none | stop |
| B | M114 | A<version_name> | write firmware version |
| B/F | M115 | none | read firmware version |
| B | M116 | none | read bootloader version |
| B/F | M117 | none | read serial number |
| F | M121 | none | write position |
| F | M130 | [T<kp>] [U<Ki>] [V<kd>] | temperature pid parameters | 
| F | M131 | none | print pwm values |
| F | M200 | [X<VAL>] [Y<VAL>] [Z<VAL>] [F<VAL>] [E<VAL>] | Define steps per mm|
| F | M206 | [X<VAL>] [Y] [Z] [F] [E] | define or print acceleration |
| F | M300 | [P<VAL>] | beep | 
| F | M400 | [A<VAL>] | define or print BEECode |
| F | M600 | none | print configuration variables |
| F | M601 | none | write configuration variables |
| F | M603 | none | calibration |
| F | M604 | [X<VAL>] [Y<VAL>] [Z<VAL>] | new absolute offset |
| F | M605 | [X<VAL>] [Y<VAL>] [Z<VAL>] | new relative offset |
| F | M607 | none | reset configuration variables |
| B/F | M609 | none | reset R2C2 |
| F | M625 | none | status |
| B | M630 | none | switchs to firmware |
| F | M638 | none | get last executed command |
| F | M639 | none | echo |
| B | M650 | A<VAL> | write firmware |
| B | M651 | none | check if firmware is OK |
| B | M652 | A<VAL> | read firmware and configuration variables |



