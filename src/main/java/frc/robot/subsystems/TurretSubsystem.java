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
    linearActuator = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightMaster")), MotorType.kBrushless);
    // turretRot = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightMaster")), MotorType.kBrushless);
    // leftFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightMaster")), MotorType.kBrushless);
    // rightFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightMaster")), MotorType.kBrushless);

    leftFlywheel.setInverted(true);

    actuatorController = linearActuator.getPIDController();
    // turretController = turretRot.getPIDController();
    // leftController = leftFlywheel.getPIDController();
    // rightController = rightFlywheel.getPIDController();

    actuatorController.setP(1.0e-5);
    actuatorController.setI(1.0e-5);
    actuatorController.setD(1.0e-5);
    actuatorController.setReference(0.0, ControlType.kPosition);
    actuatorController.setOutputRange(-0.20, 0.20);
    
  }

  @Override
  public void periodic() {
    actuatorController.setReference(20.0, ControlType.kPosition);
  }
}
