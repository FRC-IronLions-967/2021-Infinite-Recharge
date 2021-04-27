/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.values.ValuesInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Autonomous m_auto;

  private SubsystemsInstance subsystemsInst;
  private IO ioInst;
  private ValuesInstance valInst;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    // don't have a way to do this with the thread yet
    SmartDashboard.putData("Auto choices", m_chooser);
    // SmartDashboard.putNumber("maxAccel", 0.02d);
    // SmartDashboard.putNumber("scale", 0.5d);
    // SmartDashboard.putNumber("zeroTurn", 0.5d);

    ioInst = IO.getInstance();

    valInst = ValuesInstance.getInstance();

    subsystemsInst = SubsystemsInstance.getInstance();

    valInst.m_dashboardThread.putDouble("maxAccel", 0.02d);
    valInst.m_dashboardThread.putDouble("scale", 0.5d);
    valInst.m_dashboardThread.putDouble("zeroTurn", 0.5d);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("leftSpeed", subsystemsInst.m_driveSubsystem.getLeftSpeed());
    // SmartDashboard.putNumber("rightSpeed", subsystemsInst.m_driveSubsystem.getRightSpeed());
    valInst.m_dashboardThread.putDouble("leftSpeed", subsystemsInst.m_driveSubsystem.getLeftSpeed());
    valInst.m_dashboardThread.putDouble("rightSpeed", subsystemsInst.m_driveSubsystem.getRightSpeed());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    if(m_autoSelected.equals("TestAuto")) {
      m_auto = new TestAuto();
      m_auto.init();
    }
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    m_auto.periodic();
  }

  /**
   * This function is called once when teleop is enabled.
   */
  @Override
  public void teleopInit() {

    // initialize default commands for subsystems
    // probably will also initialize controls here
    CommandScheduler.getInstance().setDefaultCommand(subsystemsInst.m_driveSubsystem, new ArcadeDriveCommand());

    ioInst.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called once when the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * This function is called periodically when disabled.
   */
  @Override
  public void disabledPeriodic() {
  }

  /**
   * This function is called once when test mode is enabled.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
