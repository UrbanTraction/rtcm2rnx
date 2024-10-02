from pyrtcm import RTCMReader
with open('tests/data/debug.rtcm', 'rb') as stream:
  rtr = RTCMReader(stream)
  for raw_data, parsed_data in rtr:
    print(parsed_data)