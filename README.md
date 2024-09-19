## rtcm2rnx

Tools for converting RTCM data into RINEX and Georust data structures. Mirros RTCM conversion methods found in RTKLIB. 

Current status:
* WIP support for GPS and Galileo MSM7 to RINEX OBS
* Command line interface: `rtcm2rnx convert <path_to_rtcm_file>`
  

Next Steps:
* Build lib interface for integrating into RTCM stream processor (Tokio Async, etc.)
* Tests (reference RTKLIB?)
* Merge CLI into georust/rinex?

  
