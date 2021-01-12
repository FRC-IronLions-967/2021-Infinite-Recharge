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
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax linearActuator;
  private CANSparkMax turretRot;
  private CANSparkMax leftFlywheel;
  private CANSparkMax rightFlywheel;

  private CANPIDController actuatorController;
  private CANPIDController turretController;
  private CANPIDController leftController;
  private CANPIDController rightController;

  private CANDigitalInput actuatorForward;
  private CANDigitalInput actuatorReverse;
  private CANDigitalInput rotForward;
  private CANDigitalInput rotReverse;

  private boolean turretInitialized = false;
  private boolean actuatorInitialized = false;

  private static final double MAX_LINEAR_ACTUATOR_POS = 180.0;
  private static final double MAX_LINEAR_ACTUATOR_NEG = 0.0;
  private static final double MAX_TURRET_POS = 48.0;
  private static final double MAX_TURRET_NEG = -48.0;

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    linearActuator = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("linearActuator")), MotorType.kBrushless);
    turretRot = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("turretRot")), MotorType.kBrushless);
    leftFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("leftFlywheel")), MotorType.kBrushless);
    rightFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightFlywheel")), MotorType.kBrushless);

    actuatorForward = linearActuator.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    actuatorReverse = linearActuator.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rotForward = turretRot.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    rotReverse = turretRot.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    leftFlywheel.setInverted(false);
    rightFlywheel.setInverted(true);

    linearActuator.getEncoder().setPosition(0.0);
    linearActuator.getEncoder().setPositionConversionFactor(1.0);

    turretRot.getEncoder().setPosition(0.0);
    turretRot.getEncoder().setPositionConversionFactor(1.0);
    turretRot.setClosedLoopRampRate(1.0);

    leftFlywheel.setClosedLoopRampRate(2.0);
    rightFlywheel.setClosedLoopRampRate(2.0);

    actuatorController = linearActuator.getPIDController();
    turretController = turretRot.getPIDController();
    leftController = leftFlywheel.getPIDController();
    rightController = rightFlywheel.getPIDController();

    actuatorController.setP(1.0e-1);
    actuatorController.setI(0.0);
    actuatorController.setD(0.0);
    actuatorController.setReference(0.0, ControlType.kPosition);
    actuatorController.setOutputRange(-0.20, 0.20);

    turretController.setP(1.0e-2);
    turretController.setI(0.0);
    turretController.setD(0.0);
    turretController.setReference(0.0, ControlType.kPosition);
    turretController.setOutputRange(-0.40, 0.40);

    leftController.setP(4.0e-4);
    leftController.setI(2.0e-7);
    leftController.setD(0.0e-8);
    // leftController.setFF(1.58e-4);
    leftController.setReference(0.0, ControlType.kVelocity);
    leftController.setOutputRange(0.0, 1.0);
    
    rightController.setP(4.0e-4);
    rightController.setI(2.0e-7);
    rightController.setD(0.0e-8);
    // rightController.setFF(1.58e-4);
    rightController.setReference(0.0, ControlType.kVelocity);
    rightController.setOutputRange(0.0, 1.0);

    SmartDashboard.putNumber("Angle Setpoint", 0.0);
    SmartDashboard.putNumber("Turret Setpoint", 0.0);
    SmartDashboard.putNumber("Flywheel Setpoint", 0.0);

  }

  public void initializeTurret() {
    if(turretInitialized) return;
    while(!rotReverse.get()) {
      turretRot.set(-0.05);
    }
    turretRot.set(0.0);
    turretRot.getEncoder().setPosition(0.0);
    turretController.setReference(12.0, ControlType.kPosition);

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

  @Override
  public void periodic() {

    if(turretInitialized && actuatorInitialized) {
      double angleSet = SmartDashboard.getNumber("Angle Setpoint", 0.0);
      angleSet = (angleSet > MAX_LINEAR_ACTUATOR_POS) ? MAX_LINEAR_ACTUATOR_POS : angleSet;
      angleSet = (angleSet < MAX_LINEAR_ACTUATOR_NEG) ? MAX_LINEAR_ACTUATOR_NEG : angleSet;
      actuatorController.setReference(angleSet, ControlType.kPosition);
      // SmartDashboard.putNumber("Angle Revolutions", linearActuator.getEncoder().getPosition());

      double turretSet = SmartDashboard.getNumber("Turret Setpoint", 0.0);
      turretSet = (turretSet > MAX_TURRET_POS) ? MAX_TURRET_POS : turretSet;
      turretSet = (turretSet < MAX_TURRET_NEG) ? MAX_TURRET_NEG : turretSet;
      turretController.setReference(turretSet, ControlType.kPosition);
      // SmartDashboard.putNumber("Turret Revolutions", turretRot.getEncoder().getPosition());
      // SmartDashboard.putNumber("Turret RPM", turretRot.getEncoder().getVelocity());
    
      // double flywheelSet = SmartDashboard.getNumber("Flywheel Setpoint", 4200.0);
      double flywheelSet = 5200.0;
      CANError left = leftController.setReference(flywheelSet, ControlType.kVelocity);
      if(!left.equals(CANError.kOk)) {
        System.out.println(left.name());
      }
      SmartDashboard.putNumber("Left RPM", leftFlywheel.getEncoder().getVelocity());
      SmartDashboard.putNumber("Left Current", leftFlywheel.getOutputCurrent());
      SmartDashboard.putNumber("Left I accum", leftController.getIAccum());

      CANError right = rightController.setReference(flywheelSet, ControlType.kVelocity);
      if(!right.equals(CANError.kOk)) {
        System.out.println(right.name());
      }
      SmartDashboard.putNumber("Right RPM", rightFlywheel.getEncoder().getVelocity());
      SmartDashboard.putNumber("Right Current", rightFlywheel.getOutputCurrent());
      SmartDashboard.putNumber("Right I accum", rightController.getIAccum());
      // leftFlywheel.set(1.0);
      // rightFlywheel.set(1.0);
    } else {
      initializeActuator();
      initializeTurret();
    }

    SmartDashboard.putBoolean("actuatorForward", actuatorForward.get());
    SmartDashboard.putBoolean("actuatorReverse", actuatorReverse.get());
    SmartDashboard.putBoolean("rotForward", rotForward.get());
    SmartDashboard.putBoolean("rotReverse", rotReverse.get());
  }
}
