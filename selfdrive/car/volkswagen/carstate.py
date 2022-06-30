import numpy as np
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, BUTTON_STATES, CarControllerParams

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.buttonStates = BUTTON_STATES.copy()
    can_define = CANDefine(DBC_FILES.pq)
    self.get_can_parser = self.get_pq_can_parser
    self.update = self.update_pq
    self.shifter_values = can_define.dv["Getriebe_1"]["Waehlhebelposition__Getriebe_1_"]

  def update_pq(self, pt_cp):

    ret = car.CarState.new_message()
    # Update vehicle speed and acceleration from ABS wheel speeds.
    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["Bremse_3"]["Radgeschw__VL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__VR_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HL_4_1"],
      pt_cp.vl["Bremse_3"]["Radgeschw__HR_4_1"],
    )

    ret.vEgoRaw = float(np.mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgo < 0.1

    # Update steering angle, rate, yaw rate, and driver input torque. VW send
    # the sign/direction in a separate signal so they must be recombined.
    ret.steeringAngleDeg = pt_cp.vl["Lenkhilfe_3"]["LH3_BLW"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_BLWSign"])]
    ret.steeringRateDeg = pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit"] * (1, -1)[int(pt_cp.vl["Lenkwinkel_1"]["Lenkradwinkel_Geschwindigkeit_S"])]
    ret.steeringTorque = pt_cp.vl["Lenkhilfe_3"]["LH3_LM"] * (1, -1)[int(pt_cp.vl["Lenkhilfe_3"]["LH3_LMSign"])]
    ret.steeringPressed = abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE
    ret.yawRate = pt_cp.vl["Bremse_5"]["Giergeschwindigkeit"] * (1, -1)[int(pt_cp.vl["Bremse_5"]["Vorzeichen_der_Giergeschwindigk"])] * CV.DEG_TO_RAD

    # Verify EPS readiness to accept steering commands
    ret.steeringFault = pt_cp.vl["Lenkhilfe_2"]['LH2_Sta_HCA'] not in [3, 5, 7]

    # Update gas, brakes, and gearshift
    ret.gas = pt_cp.vl["Motor_3"]['Fahrpedal_Rohsignal'] / 100.0
    ret.gasPressed = ret.gas > 0

    ret.brake = pt_cp.vl["Bremse_5"]["Bremsdruck"] / 250.0  # FIXME: this is pressure in Bar, not sure what OP expects
    ret.brakePressed = bool(pt_cp.vl["Motor_2"]["Bremstestschalter"])

    # Update door and trunk/hatch lid open status.
    # TODO: need to locate signals for other three doors if possible
    ret.doorOpen = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Fa_Tuerkont"])

    # Update seatbelt fastened status.
    ret.seatbeltUnlatched = not bool(pt_cp.vl["Airbag_1"]["Gurtschalter_Fahrer"])

    # Update driver preference for metric. VW stores many different unit
    # preferences, including separate units for for distance vs. speed.
    # We use the speed preference for OP.
    # TODO: read PQ Einheiten here
    self.displayMetricUnits = False

    # Update ACC radar status.
    ret.cruiseState.available = bool(pt_cp.vl["GRA_Neu"]['GRA_Hauptschalt'])
    ret.cruiseState.enabled = True if pt_cp.vl["Motor_2"]['GRA_Status'] in [1, 2] else False
    ret.leftBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_li"])
    ret.rightBlinker = bool(pt_cp.vl["Gate_Komf_1"]["GK1_Blinker_re"])
    # Update ACC setpoint. When the setpoint reads as 255, the driver has not
    ret.cruiseState.speed = pt_cp.vl["Motor_2"]['Soll_Geschwindigkeit_bei_GRA_Be'] * CV.KPH_TO_MS

    # Additional safety checks performed in CarInterface.
    self.parkingBrakeSet = False #bool(pt_cp.vl["Kombi_1"]["Bremsinfo"])  # FIXME: need to include an EPB check as well
    ret.espDisabled = bool(pt_cp.vl["Bremse_1"]["ESP_Passiv_getastet"])

    return ret

  @staticmethod
  def get_pq_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("LH3_BLW", "Lenkhilfe_3"),                # Absolute steering angle
      ("LH3_BLWSign", "Lenkhilfe_3"),            # Steering angle sign
      ("LH3_LM", "Lenkhilfe_3"),                 # Absolute driver torque input
      ("LH3_LMSign", "Lenkhilfe_3"),             # Driver torque input sign
      ("LH2_Sta_HCA", "Lenkhilfe_2"),            # Steering rack HCA status
      ("Lenkradwinkel_Geschwindigkeit", "Lenkwinkel_1"),  # Absolute steering rate
      ("Lenkradwinkel_Geschwindigkeit_S", "Lenkwinkel_1"),  # Steering rate sign
      ("Radgeschw__VL_4_1", "Bremse_3"),         # ABS wheel speed, front left
      ("Radgeschw__VR_4_1", "Bremse_3"),         # ABS wheel speed, front right
      ("Radgeschw__HL_4_1", "Bremse_3"),         # ABS wheel speed, rear left
      ("Radgeschw__HR_4_1", "Bremse_3"),         # ABS wheel speed, rear right
      ("Giergeschwindigkeit", "Bremse_5"),       # Absolute yaw rate
      ("Vorzeichen_der_Giergeschwindigk", "Bremse_5"),  # Yaw rate sign
      ("Gurtschalter_Fahrer", "Airbag_1"),       # Seatbelt status, driver
      ("Gurtschalter_Beifahrer", "Airbag_1"),    # Seatbelt status, passenger
      ("Bremstestschalter", "Motor_2"),          # Brake pedal pressed (brake light test switch)
      ("Bremslichtschalter", "Motor_2"),         # Brakes applied (brake light switch)
      ("Soll_Geschwindigkeit_bei_GRA_Be", "Motor_2"), #CruiseControl Setspeed
      ("Bremsdruck", "Bremse_5"),                # Brake pressure applied
      ("Vorzeichen_Bremsdruck", "Bremse_5"),     # Brake pressure applied sign (???)
      ("Fahrpedal_Rohsignal", "Motor_3"),        # Accelerator pedal value
      ("ESP_Passiv_getastet", "Bremse_1"),       # Stability control disabled
      ("GK1_Fa_Tuerkont", "Gate_Komf_1"),        # Door open, driver
      # TODO: locate passenger and rear door states
      ("GK1_Blinker_li", "Gate_Komf_1"),         # Left turn signal on
      ("GK1_Blinker_re", "Gate_Komf_1"),         # Right turn signal on
      ("Bremsinfo", "Kombi_1"),                  # Manual handbrake applied
      ("GRA_Status", "Motor_2"),                 # ACC engagement status
      ("GRA_Hauptschalt", "GRA_Neu"),            # ACC button, on/off
      ("Waehlhebelposition__Getriebe_1_", "Getriebe_1"),  # Auto trans gear selector position)
    ]

    checks = [
      # sig_address, frequency
      ("Bremse_1", 100),          # From J104 ABS/ESP controller
      ("Bremse_3", 100),          # From J104 ABS/ESP controller
      ("Lenkhilfe_3", 100),       # From J500 Steering Assist with integrated sensors
      ("Lenkwinkel_1", 100),      # From J500 Steering Assist with integrated sensors
      ("Motor_3", 100),           # From J623 Engine control module
      ("Airbag_1", 50),           # From J234 Airbag control module
      ("Bremse_5", 50),           # From J104 ABS/ESP controller
      ("Bremse_8", 50),           # From J??? ABS/ACC controller
      ("Kombi_1", 50),            # From J285 Instrument cluster
      ("Motor_2", 50),            # From J623 Engine control module
      ("Lenkhilfe_2", 20),        # From J500 Steering Assist with integrated sensors
      ("Gate_Komf_1", 10),        # From J533 CAN gateway
      ("GRA_Neu", 50),            # From J??? steering wheel control buttons
      ("Getriebe_1", 100),
    ]

    return CANParser(DBC_FILES.pq, signals, checks, CANBUS.pt)
