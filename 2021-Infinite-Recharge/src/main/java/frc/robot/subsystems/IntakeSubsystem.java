// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO;
import frc.robot.utils.controls.XBoxController;
import frc.robot.values.ValuesInstance;

public class IntakeSubsystem extends SubsystemBase {

  private TalonSRX intakeRoller;
  private TalonSRX rearBelt;
  private TalonSRX frontBelt;

  private IO ioInst;
  private ValuesInstance valInst;

  private boolean intakeOn = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    valInst = ValuesInstance.getInstance();

    intakeRoller = new TalonSRX(valInst.m_robotMap.getIntValue("intakeRoller"));
    rearBelt = new TalonSRX(valInst.m_robotMap.getIntValue("rearBelt"));
    frontBelt = new TalonSRX(valInst.m_robotMap.getIntValue("frontBelt"));

    intakeRoller.setInverted(true);
    rearBelt.setInverted(false);
    frontBelt.setInverted(true);

    ioInst = IO.getInstance();
  }

  public void toggleIntake() {
    intakeOn = (intakeOn) ? false : true;
  }

  @Override
  public void periodic() {
    if(ioInst.getManipulatorController().isTriggerPressed(XBoxController.LEFT_TRIGGER)) {
      intakeRoller.set(ControlMode.PercentOutput, 0.60);
      rearBelt.set(ControlMode.PercentOutput, 0.50);
      // frontBelt.set(ControlMode.PercentOutput, 0.50);
    } else {
      intakeRoller.set(ControlMode.PercentOutput, 0.0);
      rearBelt.set(ControlMode.PercentOutput, 0.0);
      // frontBelt.set(ControlMode.PercentOutput, 0.0);
    }

    if(intakeOn) {
      frontBelt.set(ControlMode.PercentOutput, 0.50);
    } else {
      frontBelt.set(ControlMode.PercentOutput, 0.0);
    }
  }
}
