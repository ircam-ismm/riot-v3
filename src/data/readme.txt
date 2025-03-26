** WIFI power {-4 ; 78} <=> {-1;19.5} dBm 
-4 = -1 dBm
8 = 2 dBm
20 = 5 dBm
28 = 7 dBm
34 = 8.5 dBm
44 = 11 dBm
52 = 13 dBm
60 = 15 dBm
68 = 17 dBm
74 = 18.5 dBm
76 = 19 dBm
78 = 19.5 dBm

** Get your local magnetic declination : https://www.magnetic-declination.com/
then convert degrees-minutes-seconds to decimal degrees :
decimal degrees = degrees + minutes/60 + seconds/3600

** Board orientation and axis - Always right handedness

Sensor UP - orientation = 0
+---------------------------+
|                           |
|        Z+ (up)            |
|] USB   .--> Y+            |
|        |                  |
|        X+                 |
|                           |
+---------------------------+

Sensor UP - orientation = 1
+---------------------------+
|        Y+                 |
|        |                  |
|] USB   .--> X+            |
|        Z+ (up)            |
|                           |
|                           |
+---------------------------+

Sensor DOWN - orientation = 2
+---------------------------+
|                           |
| (-)   Z+ (up)             |
| BATT  .--> Y+             |
| (+)   |                   |
|       X+                  |
|                           |
+---------------------------+

Sensor DOWN - orientation = 3
+---------------------------+
|       Y+                  |
| (-)   |                   |
| BATT  .--> X+             |
| (+)   Z+ (up)             |
|                           |
|                           |
+---------------------------+

** Barometric pressure sensor sampling mode, accuracy and ODR
Select pressure sensor sampling mode within (default is high precision - baro=4) :
0 = BARO_ULTRA_LOW_PRECISION   	// OSRx1, T-OSRx1, ODR 25/2048, no IIR
1 = BARO_LOW_PRECISION = 1      // OSRx2, T-OSRx1, ODR 100HZ, no IIR 
2 = BARO_NORMAL_PRECISION1,     // OSRx4, T-OSRx1, ODR 50HZ, IIR COEF 3 
3 = BARO_NORMAL_PRECISION2,     // OSRx8, T-OSRx1, ODR 50HZ, IIR COEF 1 (more oversampling, less filtering) 
4 = BARO_HIGH_PRECISION,        // OSRx8, T-OSRx1, ODR 12.5HZ, IIR COEF 1
5 = BARO_ULTRA_PRECISION,       // OSRx16, T-OSRx2, ODR 25HZ, IIR COEF 3
 