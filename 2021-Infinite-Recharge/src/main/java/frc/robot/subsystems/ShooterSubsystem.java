// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.utils.controls.XBoxController;
import frc.robot.values.ValuesInstance;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax leftFlywheel;
  private CANSparkMax rightFlywheel;
  private TalonSRX feedWheel;

  private CANPIDController leftController;
  private CANPIDController rightController;

  private IO ioInst;
  private ValuesInstance valInst;

  private boolean feederOn = false;

  private double targetRPM;
  private final double maxRPM;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    ioInst = IO.getInstance();
    valInst = ValuesInstance.getInstance();

    leftFlywheel = new CANSparkMax(Integer.parseInt(valInst.m_robotMap.getValue("leftFlywheel")), MotorType.kBrushless);
    rightFlywheel = new CANSparkMax(Integer.parseInt(valInst.m_robotMap.getValue("rightFlywheel")), MotorType.kBrushless);
    feedWheel = new TalonSRX(valInst.m_robotMap.getIntValue("feedWheel"));

    leftFlywheel.setInverted(false);
    rightFlywheel.setInverted(true);
    feedWheel.setInverted(false);

    leftFlywheel.setClosedLoopRampRate(2.0);
    rightFlywheel.setClosedLoopRampRate(2.0);

    leftController = leftFlywheel.getPIDController();
    rightController = rightFlywheel.getPIDController();

    leftController.setP(valInst.m_pidValues.getDoubleValue("lFlyP"));
    leftController.setI(valInst.m_pidValues.getDoubleValue("lFlyI"));
    leftController.setD(valInst.m_pidValues.getDoubleValue("lFlyD"));
    leftController.setReference(0.0, ControlType.kVelocity);
    leftController.setOutputRange(valInst.m_pidValues.getDoubleValue("lFlyMin"), valInst.m_pidValues.getDoubleValue("lFlyMax"));
    
    rightController.setP(valInst.m_pidValues.getDoubleValue("rFlyP"));
    rightController.setI(valInst.m_pidValues.getDoubleValue("rFlyI"));
    rightController.setD(valInst.m_pidValues.getDoubleValue("rFlyD"));
    rightController.setReference(0.0, ControlType.kVelocity);
    rightController.setOutputRange(valInst.m_pidValues.getDoubleValue("rFlyMin"), valInst.m_pidValues.getDoubleValue("rFlyMax"));

    targetRPM = valInst.m_values.getDoubleValue("defaultTargetRPM");
    maxRPM = valInst.m_values.getDoubleValue("maxShooterRPM");

    SmartDashboard.putNumber("Flywheel Setpoint", 0.0);
  }

  // sets the rpm that the flywheel will target when it is told to activate
  public void setTargetRPM(double rpm) {
    targetRPM = (rpm > maxRPM) ? maxRPM : rpm;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getMaxRPM() {
    return maxRPM;
  }

  public void toggleFeeder() {
    feederOn = (feederOn) ? false : true;
  }

  @Override
  public void periodic() {

    if(ioInst.getManipulatorController().isTriggerPressed(XBoxController.RIGHT_TRIGGER)) {
      leftController.setReference(targetRPM, ControlType.kVelocity);
      rightController.setReference(targetRPM, ControlType.kVelocity);
    } else {
      leftController.setReference(0.0, ControlType.kVelocity);
      rightController.setReference(0.0, ControlType.kVelocity);
    }

    // if(ioInst.getManipulatorController().isButtonPressed("X")) {
      if(feederOn) {
      feedWheel.set(ControlMode.PercentOutput, 1.0);
    } else {
      feedWheel.set(ControlMode.PercentOutput, 0.0);
    }

    SmartDashboard.putNumber("Flywheel Setpoint", targetRPM);

  }
}
