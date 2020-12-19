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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

  /**
   * Creates a new TurretSubsystem.
   */
  public TurretSubsystem() {
    linearActuator = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("linearActuator")), MotorType.kBrushless);
    turretRot = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("turretRot")), MotorType.kBrushless);
    leftFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("leftFlywheel")), MotorType.kBrushless);
    rightFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightFlywheel")), MotorType.kBrushless);

    leftFlywheel.setInverted(true);
    rightFlywheel.setInverted(false);

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

    leftController.setP(1.0e-3);
    leftController.setI(1.0e-7);
    leftController.setD(1.0e-2);
    leftController.setReference(0.0, ControlType.kVelocity);
    leftController.setOutputRange(0.0, 1.0);
    
    rightController.setP(1.0e-3);
    rightController.setI(1.0e-7);
    rightController.setD(1.0e-2);
    rightController.setReference(0.0, ControlType.kVelocity);
    rightController.setOutputRange(0.0, 1.0);

    SmartDashboard.putNumber("Angle Setpoint", 0.0);
    SmartDashboard.putNumber("Turret Setpoint", 0.0);
    SmartDashboard.putNumber("Flywheel Setpoint", 0.0);
  }

  @Override
  public void periodic() {
    double angleSet = SmartDashboard.getNumber("Angle Setpoint", 0.0);
    actuatorController.setReference(angleSet, ControlType.kPosition);
    SmartDashboard.putNumber("Angle Revolutions", linearActuator.getEncoder().getPosition());

    double turretSet = SmartDashboard.getNumber("Turret Setpoint", 0.0);
    turretController.setReference(turretSet, ControlType.kPosition);
    SmartDashboard.putNumber("Turret Revolutions", turretRot.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret RPM", turretRot.getEncoder().getVelocity());
    
    double flywheelSet = SmartDashboard.getNumber("Flywheel Setpoint", 0.0);
    leftController.setReference(flywheelSet, ControlType.kVelocity);
    SmartDashboard.putNumber("Left RPM", leftFlywheel.getEncoder().getVelocity());
    SmartDashboard.putNumber("Left Current", leftFlywheel.getOutputCurrent());

    rightController.setReference(flywheelSet, ControlType.kVelocity);
    SmartDashboard.putNumber("Right RPM", rightFlywheel.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Current", rightFlywheel.getOutputCurrent());
  }
}
