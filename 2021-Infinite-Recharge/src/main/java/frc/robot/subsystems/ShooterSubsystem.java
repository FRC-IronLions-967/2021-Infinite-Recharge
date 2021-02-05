// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax leftFlywheel;
  private CANSparkMax rightFlywheel;

  private CANPIDController leftController;
  private CANPIDController rightController;


  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    leftFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("leftFlywheel")), MotorType.kBrushless);
    rightFlywheel = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightFlywheel")), MotorType.kBrushless);

    leftFlywheel.setInverted(false);
    rightFlywheel.setInverted(true);

    leftFlywheel.setClosedLoopRampRate(2.0);
    rightFlywheel.setClosedLoopRampRate(2.0);

    leftController = leftFlywheel.getPIDController();
    rightController = rightFlywheel.getPIDController();

    leftController.setP(Robot.m_pidValues.getDoubleValue("lFlyP"));
    leftController.setI(Robot.m_pidValues.getDoubleValue("lFlyI"));
    leftController.setD(Robot.m_pidValues.getDoubleValue("lFlyD"));
    leftController.setReference(0.0, ControlType.kVelocity);
    leftController.setOutputRange(Robot.m_pidValues.getDoubleValue("lFlyMin"), Robot.m_pidValues.getDoubleValue("lFlyMax"));
    
    rightController.setP(Robot.m_pidValues.getDoubleValue("rFlyP"));
    rightController.setI(Robot.m_pidValues.getDoubleValue("rFlyI"));
    rightController.setD(Robot.m_pidValues.getDoubleValue("rFlyD"));
    rightController.setReference(0.0, ControlType.kVelocity);
    rightController.setOutputRange(Robot.m_pidValues.getDoubleValue("rFlyMin"), Robot.m_pidValues.getDoubleValue("rFlyMax"));

    SmartDashboard.putNumber("Flywheel Setpoint", 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double flywheelSet = SmartDashboard.getNumber("Flywheel Setpoint", 0.0);

    CANError left = leftController.setReference(flywheelSet, ControlType.kVelocity);

    if(!left.equals(CANError.kOk)) {
      System.out.println(left.name());
    }

    SmartDashboard.putNumber("Left RPM", leftFlywheel.getEncoder().getVelocity());

    CANError right = rightController.setReference(flywheelSet, ControlType.kVelocity);

    if(!right.equals(CANError.kOk)) {
     System.out.println(right.name());
    }
    
    SmartDashboard.putNumber("Right RPM", rightFlywheel.getEncoder().getVelocity());
  }
}
