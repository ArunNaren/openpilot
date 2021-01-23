from cereal import car
from common.numpy_fast import clip
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC, CANBUS, NWL, MQB_LDW_MESSAGES, BUTTON_STATES, CarControllerParams
from opendbc.can.packer import CANPacker


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.mobPreEnable = False
    self.mobEnabled = False
    self.radarVin_idx = 0

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.acc_bus = CANBUS.pt if CP.networkLocation == NWL.fwdCamera else CANBUS.cam

    if CP.safetyModel == car.CarParams.SafetyModel.volkswagen:
      self.create_steering_control = volkswagencan.create_mqb_steering_control
      self.create_acc_buttons_control = volkswagencan.create_mqb_acc_buttons_control
      self.create_hud_control = volkswagencan.create_mqb_hud_control
      self.ldw_step = CarControllerParams.MQB_LDW_STEP
    elif CP.safetyModel == car.CarParams.SafetyModel.volkswagenPq:
      self.create_steering_control = volkswagencan.create_pq_steering_control
      self.create_acc_buttons_control = volkswagencan.create_pq_acc_buttons_control
      self.create_hud_control = volkswagencan.create_pq_hud_control
      self.create_braking_control = volkswagencan.create_pq_braking_control
      self.create_gas_control = volkswagencan.create_pq_pedal_control
      self.create_awv_control = volkswagencan.create_pq_awv_control
      self.ldw_step = CarControllerParams.PQ_LDW_STEP

    self.hcaSameTorqueCount = 0
    self.hcaEnabledFrameCount = 0
    self.graButtonStatesToSend = None
    self.graMsgSentCount = 0
    self.graMsgStartFramePrev = 0
    self.graMsgBusCounterPrev = 0

    self.steer_rate_limited = False

  def update(self, enabled, CS, frame, actuators, visual_alert, audible_alert, leftLaneVisible, rightLaneVisible):
    """ Controls thread """

    P = CarControllerParams

    # Send CAN commands.
    can_sends = []

    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare HCA_01 Heading Control Assist messages with steering torque.    #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The factory camera sends at 50Hz while steering and 1Hz when not. When
    # OP is active, Panda filters HCA_01 from the factory camera and OP emits
    # HCA_01 at 50Hz. Rate switching creates some confusion in Cabana and
    # doesn't seem to add value at this time. The rack will accept HCA_01 at
    # 100Hz if we want to control at finer resolution in the future.
    if frame % P.HCA_STEP == 0:

      # FAULT AVOIDANCE: HCA must not be enabled at standstill. Also stop
      # commanding HCA if there's a fault, so the steering rack recovers.
      if enabled and not (CS.out.standstill or CS.steeringFault):

        # FAULT AVOIDANCE: Requested HCA torque must not exceed 3.0 Nm. This
        # is inherently handled by scaling to STEER_MAX. The rack doesn't seem
        # to care about up/down rate, but we have some evidence it may do its
        # own rate limiting, and matching OP helps for accurate tuning.
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer

        # FAULT AVOIDANCE: HCA must not be enabled for >360 seconds. Sending
        # a single frame with HCA disabled is an effective workaround.
        if apply_steer == 0:
          # We can usually reset the timer for free, just by disabling HCA
          # when apply_steer is exactly zero, which happens by chance during
          # many steer torque direction changes. This could be expanded with
          # a small dead-zone to capture all zero crossings, but not seeing a
          # major need at this time.
          hcaEnabled = False
          self.hcaEnabledFrameCount = 0
        else:
          self.hcaEnabledFrameCount += 1
          if self.hcaEnabledFrameCount >= 118 * (100 / P.HCA_STEP):  # 118s
            # The Kansas I-70 Crosswind Problem: if we truly do need to steer
            # in one direction for > 360 seconds, we have to disable HCA for a
            # frame while actively steering. Testing shows we can just set the
            # disabled flag, and keep sending non-zero torque, which keeps the
            # Panda torque rate limiting safety happy. Do so 3x within the 360
            # second window for safety and redundancy.
            hcaEnabled = False
            self.hcaEnabledFrameCount = 0
          else:
            hcaEnabled = True
            # FAULT AVOIDANCE: HCA torque must not be static for > 6 seconds.
            # This is to detect the sending camera being stuck or frozen. OP
            # can trip this on a curve if steering is saturated. Avoid this by
            # reducing torque 0.01 Nm for one frame. Do so 3x within the 6
            # second period for safety and redundancy.
            if self.apply_steer_last == apply_steer:
              self.hcaSameTorqueCount += 1
              if self.hcaSameTorqueCount > 1.9 * (100 / P.HCA_STEP):  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.hcaSameTorqueCount = 0
            else:
              self.hcaSameTorqueCount = 0

      else:
        # Continue sending HCA_01 messages, with the enable flags turned off.
        hcaEnabled = False
        apply_steer = 0

      self.apply_steer_last = apply_steer
      idx = (frame / P.HCA_STEP) % 16
      can_sends.append(self.create_steering_control(self.packer_pt, CANBUS.pt, apply_steer, idx, hcaEnabled))
      #can_sends.append(volkswagencan.create_pq_dsr_control(self.packer_pt, CANBUS.br, apply_steer, idx, hcaEnabled))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare PQ_MOB for sending the braking message                          #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.MOB_STEP == 0) and CS.CP.enableGasInterceptor:
      mobEnabled = self.mobEnabled
      mobPreEnable = self.mobPreEnable
      # TODO make sure we use the full 8190 when calculating braking.
      apply_brake = actuators.brake * 1200
      stopping_wish = False

      if enabled:
        if (apply_brake < 40):
          apply_brake = 0
        if apply_brake > 0:
          if not mobEnabled:
            mobEnabled = True
            apply_brake = 0
          elif not mobPreEnable:
            mobPreEnable = True
            apply_brake = 0
          elif apply_brake > 1199:
            apply_brake = 1200
            CS.brake_warning = True
          if CS.currentSpeed < 5.6:
            stopping_wish = True
        else:
          mobPreEnable = False
          mobEnabled = False
      else:
        apply_brake = 0
        mobPreEnable = False
        mobEnabled = False

      idx = (frame / P.MOB_STEP) % 16
      self.mobPreEnable = mobPreEnable
      self.mobEnabled = mobEnabled
      can_sends.append(self.create_braking_control(self.packer_pt, CANBUS.br, apply_brake, idx, mobEnabled, mobPreEnable, stopping_wish))

      # --------------------------------------------------------------------------
      #                                                                         #
      # Prepare PQ_MOB for sending the braking message                          #
      #                                                                         #
      #                                                                         #
      # --------------------------------------------------------------------------
      if (frame % P.AWV_STEP == 0) and CS.CP.enableGasInterceptor:
        green_led = 1 if enabled else 0
        orange_led = 1 if self.mobPreEnable and self.mobEnabled else 0
        if enabled:
          braking_working = 0 if (CS.ABSWorking == 0) else 5
        else:
          braking_working = 0

        idx = (frame / P.MOB_STEP) % 16

        can_sends.append(
          self.create_awv_control(self.packer_pt, CANBUS.pt, idx, orange_led, green_led, braking_working))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare GAS_COMMAND for sending towards Pedal                           #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    if (frame % P.GAS_STEP == 0) and CS.CP.enableGasInterceptor:
      apply_gas = 0
      if enabled:
        apply_gas = clip(actuators.gas, 0., 1.)

      can_sends.append(self.create_gas_control(self.packer_pt, CANBUS.cam, apply_gas, frame // 2))

    # --------------------------------------------------------------------------
    #                                                                         #
    # Prepare VIN_MESSAGE for sending towards Panda                           #
    #                                                                         #
    #                                                                         #
    # --------------------------------------------------------------------------
    # if using radar, we need to send the VIN
    if CS.useTeslaRadar and (frame % 100 == 0):
      can_sends.append(
        volkswagencan.create_radar_VIN_msg(self.radarVin_idx, CS.radarVIN, 2, 0x4A0, CS.useTeslaRadar,
                                            CS.radarPosition,
                                            CS.radarEpasType))
      self.radarVin_idx += 1
      self.radarVin_idx = self.radarVin_idx % 3

    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare LDW_02 HUD messages with lane borders, confidence levels, and   #
    # the LKAS status LED.                                                    #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The factory camera emits this message at 10Hz. When OP is active, Panda
    # filters LDW_02 from the factory camera and OP emits LDW_02 at 10Hz.

    if frame % self.ldw_step == 0:
      hcaEnabled = True if enabled and not CS.out.standstill else False

      if visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired:
        hud_alert = MQB_LDW_MESSAGES["laneAssistTakeOverSilent"]
      else:
        hud_alert = MQB_LDW_MESSAGES["none"]

      can_sends.append(self.create_hud_control(self.packer_pt, CANBUS.pt, hcaEnabled,
                                                            CS.out.steeringPressed, hud_alert, leftLaneVisible,
                                                            rightLaneVisible, CS.ldw_lane_warning_left,
                                                            CS.ldw_lane_warning_right, CS.ldw_side_dlc_tlc,
                                                            CS.ldw_dlc, CS.ldw_tlc))

    #--------------------------------------------------------------------------
    #                                                                         #
    # Prepare GRA_ACC_01 ACC control messages with button press events.       #
    #                                                                         #
    #--------------------------------------------------------------------------

    # The car sends this message at 33hz. OP sends it on-demand only for
    # virtual button presses.
    #
    # First create any virtual button press event needed by openpilot, to sync
    # stock ACC with OP disengagement, or to auto-resume from stop.

    if frame > self.graMsgStartFramePrev + P.GRA_VBP_STEP:
      if not enabled and CS.out.cruiseState.enabled:
        # Cancel ACC if it's engaged with OP disengaged.
        self.graButtonStatesToSend = BUTTON_STATES.copy()
        self.graButtonStatesToSend["cancel"] = True
      elif enabled and CS.out.standstill:
        # Blip the Resume button if we're engaged at standstill.
        # FIXME: This is a naive implementation, improve with visiond or radar input.
        # A subset of MQBs like to "creep" too aggressively with this implementation.
        self.graButtonStatesToSend = BUTTON_STATES.copy()
        self.graButtonStatesToSend["resumeCruise"] = True
      elif enabled and CS.out.cruiseState.enabled and CS.CP.enableGasInterceptor:
        self.graButtonStatesToSend = BUTTON_STATES.copy()
        self.graButtonStatesToSend["cancel"] = True

    # OP/Panda can see this message but can't filter it when integrated at the
    # R242 LKAS camera. It could do so if integrated at the J533 gateway, but
    # we need a generalized solution that works for either. The message is
    # counter-protected, so we need to time our transmissions very precisely
    # to achieve fast and fault-free switching between message flows accepted
    # at the J428 ACC radar.
    #
    # Example message flow on the bus, frequency of 33Hz (GRA_ACC_STEP):
    #
    # CAR: 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  3  4  5  6
    # EON:        3  4  5  6  7  8  9  A  B  C  D  E  F  0  1  2  GG^
    #
    # If OP needs to send a button press, it waits to see a GRA_ACC_01 message
    # counter change, and then immediately follows up with the next increment.
    # The OP message will be sent within about 1ms of the car's message, which
    # is about 2ms before the car's next message is expected. OP sends for an
    # arbitrary duration of 16 messages / ~0.5 sec, in lockstep with each new
    # message from the car.
    #
    # Because OP's counter is synced to the car, J428 immediately accepts the
    # OP messages as valid. Further messages from the car get discarded as
    # duplicates without a fault. When OP stops sending, the extra time gap
    # (GG) to the next valid car message is less than 1 * GRA_ACC_STEP. J428
    # tolerates the gap just fine and control returns to the car immediately.

    if CS.graMsgBusCounter != self.graMsgBusCounterPrev:
      self.graMsgBusCounterPrev = CS.graMsgBusCounter
      if self.graButtonStatesToSend is not None:
        if self.graMsgSentCount == 0:
          self.graMsgStartFramePrev = frame
        idx = (CS.graMsgBusCounter + 1) % 16
        can_sends.append(self.create_acc_buttons_control(self.packer_pt, CANBUS.br, self.graButtonStatesToSend, CS, idx))
        self.graMsgSentCount += 1
        if self.graMsgSentCount >= P.GRA_VBP_COUNT:
          self.graButtonStatesToSend = None
          self.graMsgSentCount = 0

    return can_sends
