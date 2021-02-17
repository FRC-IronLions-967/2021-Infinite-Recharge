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
import frc.robot.Robot;
import frc.robot.commands.*;
import frc.robot.values.CustomVisionValues;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax linearActuator;
  private CANSparkMax turretRot;

  private CANPIDController actuatorController;
  private CANPIDController turretController;

  private CANDigitalInput actuatorForward;
  private CANDigitalInput actuatorReverse;
  private CANDigitalInput rotForward;
  private CANDigitalInput rotReverse;

  private CustomVisionValues visionValues;
  private double prevTx = Double.MAX_VALUE;
  private double prevTy = Double.MAX_VALUE;
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

  private final double MAX_LINEAR_ACTUATOR_POS;
  private final double MAX_LINEAR_ACTUATOR_NEG;
  private final double MAX_TURRET_POS;
  private final double MAX_TURRET_NEG;

  private final double DEG_TO_ROT;
  private final double ROT_TO_DEG;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    linearActuator = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("linearActuator")), MotorType.kBrushless);
    turretRot = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("turretRot")), MotorType.kBrushless);

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

    actuatorController.setP(Robot.m_pidValues.getDoubleValue("actuatorP"));
    actuatorController.setI(Robot.m_pidValues.getDoubleValue("actuatorI"));
    actuatorController.setD(Robot.m_pidValues.getDoubleValue("actuatorD"));
    actuatorController.setReference(0.0, ControlType.kPosition);
    actuatorController.setOutputRange(Robot.m_pidValues.getDoubleValue("actuatorMin"), Robot.m_pidValues.getDoubleValue("actuatorMax"));

    turretController.setP(Robot.m_pidValues.getDoubleValue("turretP"));
    turretController.setI(Robot.m_pidValues.getDoubleValue("turretI"));
    turretController.setD(Robot.m_pidValues.getDoubleValue("turretD"));
    turretController.setReference(0.0, ControlType.kPosition);
    turretController.setOutputRange(Robot.m_pidValues.getDoubleValue("turretMin"), Robot.m_pidValues.getDoubleValue("turretMax"));

    SmartDashboard.putNumber("Angle Setpoint", 0.0);
    SmartDashboard.putNumber("Turret Setpoint", 0.0);
    SmartDashboard.putBoolean("Auto Tracking", false);

    // visionValues = new CustomVisionValues("target");
    targetTimeout = 0;

    MAX_LINEAR_ACTUATOR_POS = Robot.m_values.getDoubleValue("maxLinearActuatorPos");
    MAX_LINEAR_ACTUATOR_NEG = Robot.m_values.getDoubleValue("maxLinearActuatorNeg");

    MAX_TURRET_POS = Robot.m_values.getDoubleValue("maxTurretPos");
    MAX_TURRET_NEG = Robot.m_values.getDoubleValue("maxTurretNeg");

    DEG_TO_ROT = Robot.m_values.getDoubleValue("turretDegToRot");
    ROT_TO_DEG = Robot.m_values.getDoubleValue("turretRotToDeg");
  }

  public void initializeTurret() {
    if(turretInitialized) return;
    while(!rotReverse.get()) {
      turretRot.set(-0.05);
    }
    turretRot.set(0.0);
    turretRot.getEncoder().setPosition(-48.0);
    turretController.setReference(0.0, ControlType.kPosition);

    turretInitialized = true;
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
  }

  public void enableAutoTracking() {
    autoTrackEnabled = true;
  }

  public void disableAutoTracking() {
    turretRot.set(0.0);
    turretSet = turretRot.getEncoder().getPosition() * ROT_TO_DEG;
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
  }

  public void moveTurret(double newAngle) {
    if(Math.abs(newAngle) > MAX_TURRET_POS) newAngle %= MAX_TURRET_POS;
    if(newAngle < MAX_TURRET_NEG) newAngle += MAX_TURRET_POS;

    turretSet = newAngle;

    turretController.setReference(turretSet * DEG_TO_ROT, ControlType.kPosition);
  }

  @Override
  public void periodic() {

    if(turretInitialized && actuatorInitialized) {

      // this seems bad but idk
      if(autoTrackEnabled) {
        if(visionValues.hasTarget()) {
          targetTimeout = 0;
          // both constants here are arbitrary and need to be tuned
          // the first is the acceptable margin of error in tx, and the second is checking to see if tx has actually changed before telling the turret to move more
            if(Math.abs(visionValues.getTX()) > 3.0 && Math.abs(visionValues.getTX()) - Math.abs(prevTx) < -2.0) {
              // need to double check that tx is actually signed
              // this constant is also arbitrary and will need to be tuned
              // this may also end up being related to distance as well, and may need to factor that in
              // this should turn by ~7.5° per pixel of offset
              turretSet += visionValues.getTX() * 2.0;
              turretController.setReference(turretSet, ControlType.kPosition);
            }
          } else {
            // we don't have a target in sight, so move the turret within its range of motion to find one
            // at 30 fps this is one second
            if(++targetTimeout > 30){
              if(!rotReverse.get()) {
                turretRot.set(-0.05);
              } else if(!rotForward.get()) {
                turretRot.set(0.05);
              }
            } else {
              turretRot.set(0.0);
            }
          }

      } else {

        actuatorController.setReference(angleSet, ControlType.kPosition);

        double newAngle = SmartDashboard.getNumber("Turret Setpoint", turretSet);
        moveTurret(newAngle);

      }

    } else {
      CommandScheduler.getInstance().schedule(new InitializeActuatorCommand(), new InitializeTurretCommand());
    }


    autoTrackEnabled = SmartDashboard.getBoolean("Auto Tracking", false);
  }
}
