/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.commands.*;
import frc.robot.values.CustomVisionValues;
import frc.robot.values.ValuesInstance;

public class TurretSubsystem extends SubsystemBase {
  private ValuesInstance valInst;
  private IO io;

  private CANSparkMax linearActuator;
  private CANSparkMax turretRot;

  private CANPIDController actuatorController;
  private CANPIDController turretController;

  private CANDigitalInput actuatorForward;
  private CANDigitalInput actuatorReverse;
  private CANDigitalInput rotForward;
  private CANDigitalInput rotReverse;

  private CustomVisionValues visionValues;
  // private double prevTx = Double.MAX_VALUE;
  // private double prevTy = Double.MAX_VALUE;
  // this stores the number of frames since the last time the target was seen
  // when this gets above a given threshold you can start searching for the target
  private int targetTimeout;

  private double angleSet;
  private double turretSet;

  private boolean turretInitialized = false;
  private boolean actuatorInitialized = false;

  // if true, the subsystem automatically tracks the target
  // if false, the subsystem will use angles manually set by the user
  private boolean autoTrackEnabled = false;

  private boolean hitMax = false;

  private boolean manualInitComplete = false;

  private final double MAX_LINEAR_ACTUATOR_POS;
  private final double MAX_LINEAR_ACTUATOR_NEG;
  private final double MAX_TURRET_POS;
  private final double MAX_TURRET_NEG;

  private final double DEG_TO_ROT;
  private final double ROT_TO_DEG;

  private static TurretSubsystemStates state = TurretSubsystemStates.UNINITIALIZED;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    valInst = ValuesInstance.getInstance();
    io = IO.getInstance();

    linearActuator = new CANSparkMax(Integer.parseInt(valInst.m_robotMap.getValue("linearActuator")), MotorType.kBrushless);
    turretRot = new CANSparkMax(Integer.parseInt(valInst.m_robotMap.getValue("turretRot")), MotorType.kBrushless);

    actuatorForward = linearActuator.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    actuatorReverse = linearActuator.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rotForward = turretRot.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rotReverse = turretRot.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    linearActuator.getEncoder().setPosition(0.0);
    linearActuator.getEncoder().setPositionConversionFactor(1.0);

    turretRot.getEncoder().setPosition(0.0);
    turretRot.getEncoder().setPositionConversionFactor(1.0);
    turretRot.setClosedLoopRampRate(1.0);

    actuatorController = linearActuator.getPIDController();
    turretController = turretRot.getPIDController();

    actuatorController.setP(valInst.m_pidValues.getDoubleValue("actuatorP"));
    actuatorController.setI(valInst.m_pidValues.getDoubleValue("actuatorI"));
    actuatorController.setD(valInst.m_pidValues.getDoubleValue("actuatorD"));
    actuatorController.setReference(0.0, ControlType.kPosition);
    actuatorController.setOutputRange(valInst.m_pidValues.getDoubleValue("actuatorMin"), valInst.m_pidValues.getDoubleValue("actuatorMax"));

    turretController.setP(valInst.m_pidValues.getDoubleValue("turretP"));
    turretController.setI(valInst.m_pidValues.getDoubleValue("turretI"));
    turretController.setD(valInst.m_pidValues.getDoubleValue("turretD"));
    turretController.setReference(0.0, ControlType.kPosition);
    turretController.setOutputRange(valInst.m_pidValues.getDoubleValue("turretMin"), valInst.m_pidValues.getDoubleValue("turretMax"));

    SmartDashboard.putNumber("Angle Setpoint", 0.0);
    SmartDashboard.putNumber("Turret Setpoint", 0.0);
    SmartDashboard.putBoolean("Auto Tracking", false);

    visionValues = new CustomVisionValues("target");
    targetTimeout = 0;

    MAX_LINEAR_ACTUATOR_POS = valInst.m_values.getDoubleValue("maxLinearActuatorPos");
    MAX_LINEAR_ACTUATOR_NEG = valInst.m_values.getDoubleValue("maxLinearActuatorNeg");

    MAX_TURRET_POS = valInst.m_values.getDoubleValue("maxTurretPos");
    MAX_TURRET_NEG = valInst.m_values.getDoubleValue("maxTurretNeg");

    DEG_TO_ROT = valInst.m_values.getDoubleValue("turretDegToRot");
    ROT_TO_DEG = valInst.m_values.getDoubleValue("turretRotToDeg");
  }

  public void initializeTurret() {
    if(turretInitialized) return;
    while(!rotForward.get()) {
      turretRot.set(0.1);
    }
    turretRot.set(0.0);
    turretRot.getEncoder().setPosition(450.0);
    turretController.setReference(450.0, ControlType.kPosition);

    SmartDashboard.putNumber("Turret Setpoint", 360.0);

    rotReverse.enableLimitSwitch(false);
    rotForward.enableLimitSwitch(false);

    turretInitialized = true;
    System.out.println("Turret Initialized");

    state = TurretSubsystemStates.UNINITIALIZED;
  }

  public void initializeActuator() {
    if(actuatorInitialized) return;
    while(!actuatorReverse.get()) {
      linearActuator.set(-0.12);
    }
    linearActuator.set(0.0);
    linearActuator.getEncoder().setPosition(-10.0);
    actuatorController.setReference(0.0, ControlType.kPosition);

    actuatorReverse.enableLimitSwitch(true);
    actuatorForward.enableLimitSwitch(true);
    
    actuatorInitialized = true;
    System.out.println("Actuator Initialized");

    state = TurretSubsystemStates.UNINITIALIZED;
  }

  public void enableAutoTracking() {
    autoTrackEnabled = true;
  }

  public void disableAutoTracking() {
    turretRot.set(0.0);
    turretSet = turretRot.getEncoder().getPosition() * ROT_TO_DEG;
    turretController.setOutputRange(valInst.m_pidValues.getDoubleValue("turretMin"), valInst.m_pidValues.getDoubleValue("turretMax"));
    turretController.setReference(turretSet, ControlType.kPosition);
    SmartDashboard.putNumber("Turret Setpoint", turretSet);
    autoTrackEnabled = false;
  }

  public boolean isAutoTrackEnabled() {
    return autoTrackEnabled;
  }

  public void changeAngle(double delta) {
    angleSet = (angleSet > MAX_LINEAR_ACTUATOR_POS) ? MAX_LINEAR_ACTUATOR_POS : angleSet;
    angleSet = (angleSet < MAX_LINEAR_ACTUATOR_NEG) ? MAX_LINEAR_ACTUATOR_NEG : angleSet;
    angleSet += delta;

    actuatorController.setReference(angleSet, ControlType.kPosition);
  }

  public void moveTurret(double newAngle) {
    if(Math.abs(newAngle) > MAX_TURRET_POS) newAngle %= MAX_TURRET_POS;
    if(newAngle < MAX_TURRET_NEG) newAngle += MAX_TURRET_POS;

    turretSet = newAngle;

    turretController.setReference(turretSet * DEG_TO_ROT, ControlType.kPosition);
    System.out.println(turretSet * DEG_TO_ROT);
  }

  @Override
  public void periodic() {

    switch(state) {

      case UNINITIALIZED:
        if(!turretInitialized) state = TurretSubsystemStates.TURRET_INITIALIZING;
        else if(!actuatorInitialized) state = TurretSubsystemStates.ACTUATOR_INITIALIZING;
        else state = TurretSubsystemStates.MANUAL_CONTROL;
        break;

      case TURRET_INITIALIZING:
        if(!manualInitComplete) turretRot.set(io.getManipulatorController().getLeftStickX());
        else CommandScheduler.getInstance().schedule(new InitializeTurretCommand());

        manualInitComplete = io.getManipulatorController().isButtonPressed("SELECT");
        break;

      case ACTUATOR_INITIALIZING:
        CommandScheduler.getInstance().schedule(new InitializeActuatorCommand());
        break;

      case MANUAL_CONTROL:
        actuatorController.setReference(angleSet, ControlType.kPosition);

        turretSet = SmartDashboard.getNumber("Turret Setpoint", turretSet);
        if(turretRot.getEncoder().getPosition() - turretSet * DEG_TO_ROT > 3.0) {
          turretRot.set((Math.abs((turretSet * DEG_TO_ROT) - turretRot.getEncoder().getPosition()) > 30.0) ? -0.20 : -0.05);
        } else if(turretRot.getEncoder().getPosition() - turretSet * DEG_TO_ROT < -3.0) {
          turretRot.set((Math.abs((turretSet * DEG_TO_ROT) - turretRot.getEncoder().getPosition()) > 30.0) ? 0.20 : 0.05);
        } else {
          turretRot.set(0.0);
        }

        if(autoTrackEnabled) state = TurretSubsystemStates.AUTO_TRACKING;
        break;

      case AUTO_TRACKING:
      // this seems bad but idk
      if(visionValues.doesVisionTableExist()) {
        turretController.setOutputRange(0.0, 0.0);

        if(visionValues.hasTarget()) {
          targetTimeout = 0;
          // both constants here are arbitrary and need to be tuned
          // the first is the acceptable margin of error in tx, and the second is checking to see if tx has actually changed before telling the turret to move more
            if(Math.abs(visionValues.getTX()) > 5.0) {
              if(Math.abs(visionValues.getTX()) < 25.0) {
                if(visionValues.getTX() > 0.0) {
                  turretRot.set(-0.05);
                } else {
                  turretRot.set(0.05);
                }
              } else {
                if(visionValues.getTX() > 0.0) {
                  turretRot.set(-0.15);
                } else {
                  turretRot.set(0.15);
                }
              }
            } else {
              turretRot.set(0.0);
            }
          } else {
            SmartDashboard.putNumber("targetTimeout", targetTimeout);
            // we don't have a target in sight, so move the turret within its range of motion to find one
            if(++targetTimeout > 50) {
              if(!hitMax) {
                turretRot.set(0.25);
                if(turretRot.getEncoder().getPosition() >= 450.0) hitMax = true;
              } else {
                turretRot.set(-0.25);
                if(turretRot.getEncoder().getPosition() <= 0.0) hitMax = false;
              }
            } else {
              turretRot.set(0.0);
            }
          }
        }

        if(!autoTrackEnabled) state = TurretSubsystemStates.MANUAL_CONTROL;
        break;
    }

    // autoTrackEnabled = SmartDashboard.getBoolean("Auto Tracking", autoTrackEnabled);

    SmartDashboard.putBoolean("Auto Tracking", autoTrackEnabled);
    SmartDashboard.putNumber("Angle Setpoint", angleSet);
    SmartDashboard.putNumber("Turret Encoder", turretRot.getEncoder().getPosition());
    SmartDashboard.putBoolean("turretReverse", rotReverse.get());
    SmartDashboard.putBoolean("turretForward", rotForward.get());
    SmartDashboard.putBoolean("Turret Initialized", turretInitialized);
    SmartDashboard.putBoolean("Actuator Initialized", actuatorInitialized);
    }
}