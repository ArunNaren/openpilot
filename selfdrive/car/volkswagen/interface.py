from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.volkswagen.values import CAR, CANBUS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

    self.displayMetricUnitsPrev = None
    # Alias Extended CAN parser to PT/CAM parser, based on detected network location
    self.cp_ext = self.cp_cam

    self.ext_bus = CANBUS.pt
    self.cp_ext = self.cp

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0


  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "volkswagen"
    ret.radarOffCan = True

    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.volkswagenPq)]

    # Global lateral tuning defaults, can be overridden per-vehicle

    ret.steerActuatorDelay = 0.1
    ret.steerRateCost = 1.0
    ret.steerLimitTimer = 0.4
    ret.steerRatio = 15.6  # Let the params learner figure this out
    tire_stiffness_factor = 1.0  # Let the params learner figure this out
    ret.lateralTuning.pid.kf = 0.00006

    # Per-chassis tuning values, override tuning defaults here if desired

    if candidate == CAR.GOLF_MK6:
      # Averages of all 1K/5K/AJ Golf variants
      ret.mass = 1379 + STD_CARGO_KG
      ret.wheelbase = 2.58
      ret.minSteerSpeed = 20 * CV.KPH_TO_MS  # May be lower depending on model-year/EPS FW
      ret.enableGasInterceptor = False

      # OP LONG parameters
      ret.openpilotLongitudinalControl = False
      ret.longitudinalTuning.deadzoneBP = [0.]
      ret.longitudinalTuning.deadzoneV = [0.]
      ret.longitudinalTuning.kpBP = [5., 35.]
      ret.longitudinalTuning.kpV = [2.8, 1.5]
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.36]

      # PQ lateral tuning HCA_Status 7
      ret.lateralTuning.pid.kpBP = [0., 14., 20.]
      ret.lateralTuning.pid.kiBP = [0., 14., 20.]
      ret.lateralTuning.pid.kpV = [0.12, 0.135, 0.147]
      ret.lateralTuning.pid.kiV = [0.09, 0.10, 0.11]

    else:
      raise ValueError(f"unsupported car {candidate}")

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.centerToFront = ret.wheelbase * 0.45
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)
    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    buttonEvents = []

    # Process the most recent CAN message traffic, and check for validity
    # The camera CAN has no signals we use at this time, but we process it
    # anyway so we can test connectivity with can_valid
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_ext, self.CP.transmissionType)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # Check for and process state-change events (button press or release) from
    # the turn stalk switch or ACC steering wheel/control stalk buttons.
    for button in self.CS.buttonStates:
      if self.CS.buttonStates[button] != self.buttonStatesPrev[button]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = button
        be.pressed = self.CS.buttonStates[button]
        buttonEvents.append(be)

    events = self.create_common_events(ret, extra_gears=[GearShifter.eco, GearShifter.sport, GearShifter.manumatic])

    # Vehicle health and operation safety checks
    if self.CS.parkingBrakeSet:
      events.add(EventName.parkBrake)

    # Low speed steer alert hysteresis logic
    if self.CP.minSteerSpeed > 0. and ret.vEgo < (self.CP.minSteerSpeed + 1.):
      self.low_speed_alert = True
    elif ret.vEgo > (self.CP.minSteerSpeed + 2.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    ret.events = events.to_msg()
    ret.buttonEvents = buttonEvents

    # update previous car states
    self.displayMetricUnitsPrev = self.CS.displayMetricUnits
    self.buttonStatesPrev = self.CS.buttonStates.copy()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    hud_control = c.hudControl
    ret = self.CC.update(c.enabled, self.CS, self.frame, c.actuators)
    self.frame += 1
    return ret
