## rtcmlib

Library for converting RTCM data into RINEX and Georust data structures. Mirrors RTCM conversion methods found in RTKLIB. 

Current status:
* WIP support for GPS, Galileo, BeiDou MSM4 and MSM7 to RINEX OBS
* Command line interface: `rtcm2rnx convert <path_to_rtcm_file>`
* Test framework using rtklib (via [rtklib-ffi](https://github.com/kpwebb/rtklib-ffi) buildgen import) 
  
