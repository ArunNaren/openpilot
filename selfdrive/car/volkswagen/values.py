from collections import defaultdict
from typing import Dict

from cereal import car
from selfdrive.car import dbc_dict

class CarControllerParams:
  HCA_STEP = 2                   # HCA_01 message frequency 50Hz
  # Observed documented MQB limits: 3.00 Nm max, rate of change 5.00 Nm/sec.
  # Limiting rate-of-change based on real-world testing and Comma's safety
  # requirements for minimum time to lane departure.
  STEER_MAX = 300                # Max heading control assist torque 3.00 Nm
  STEER_DELTA_UP = 4             # Max HCA reached in 1.50s (STEER_MAX / (50Hz * 1.50))
  STEER_DELTA_DOWN = 10          # Min HCA reached in 0.60s (STEER_MAX / (50Hz * 0.60))
  STEER_DRIVER_ALLOWANCE = 80
  STEER_DRIVER_MULTIPLIER = 3    # weight driver torque heavily
  STEER_DRIVER_FACTOR = 1        # from dbc


  # pedal lookups, only for Volt
  MAX_GAS = 600  # Only a safety limit
  ZERO_GAS = 227
  MAX_BRAKE = 350  # Should be around 3.5m/s^2, including regen
  GAS_LOOKUP_BP = [0., 2.0]
  GAS_LOOKUP_V = [ZERO_GAS, MAX_GAS]
  BRAKE_LOOKUP_BP = [-4., 0.]
  BRAKE_LOOKUP_V = [MAX_BRAKE, 0]

class CANBUS:
  pt = 1
  br = 1
  cam = 1

class DBC_FILES:
  pq = "vw_golf_mk4"  # Used for all cars with PQ25/PQ35/PQ46/NMS-style CAN messaging

DBC = defaultdict(lambda: dbc_dict(DBC_FILES.pq, None))  # type: Dict[str, Dict[str, str]]

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

class CAR:
  GOLF_MK6 = "VOLKSWAGEN GOLF 6TH GEN"              # Chassis 5G/AU/BA/BE, Mk6 VW Golf and variants

FINGERPRINTS = {
  CAR.GOLF_MK6: [
    {1056: 8, 1088: 8, 1096: 5, 1152: 8, 1160: 8, 1162: 8, 1164: 8, 1175: 8, 1184: 8, 1192: 8, 1306: 8, 1312: 8, 1344: 8, 1352: 3, 1360: 8, 1386: 8, 1392: 5, 1394: 1, 1408: 8, 1416: 8, 1420: 8, 1423: 8, 1440: 8, 1472: 8, 1488: 8, 1490: 8, 1494: 2, 1500: 8, 1504: 8, 1654: 2, 174: 8, 1754: 4, 1824: 7, 1827: 7, 194: 8, 2000: 8, 208: 6, 416: 8, 428: 8, 640: 8, 644: 6, 648: 8, 672: 8, 80: 4, 800: 8, 896: 8, 906: 4, 912: 8, 914: 8, 919: 8, 928: 8, 976: 6, 978: 7, 980: 4},
  ],
}

FW_VERSIONS = {
}
