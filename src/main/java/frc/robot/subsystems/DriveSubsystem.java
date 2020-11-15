/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO test all features and review code for potential errors that could arise during operation

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.IO;
import frc.robot.Robot;
import frc.robot.utils.Utils;
import frc.robot.utils.kalman.BasicPosKalman;
import frc.robot.values.LookupGenerator;
import frc.robot.values.LookupType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.apache.commons.math3.linear.*;

import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase implements Subsystem {
  /**
   * Creates a new DriveSubsystem.
   */

  private double v = 0.0;

  private double avgAcc = 0.0;
  
  public CANSparkMax rightMaster;
  public CANSparkMax rightSlave;
  public CANSparkMax leftMaster;
  public CANSparkMax leftSlave;

  private IO io;

  //Drive lookup table(might be automatically generated in the future).  Update: is now replaced by generated table.
  private double lookup[] = {0, 0, 0,  0.1, 0.10009, 0.10036, 0.10081, 
		0.10144, 0.10225, 0.10324, 0.10441, 0.10576, 0.10729, 
		0.109, 0.11089, 0.11296, 0.11521, 0.11764, 0.12025, 
		0.12304, 0.12601, 0.12916, 0.13249, 0.136, 0.13969, 0.14356, 
		0.14761, 0.15184, 0.15625, 0.16084, 0.16561, 0.17056, 0.17569, 0.181, 
		0.18649, 0.19216, 0.19801, 0.20404, 0.21025, 0.21664, 0.22321, 
		0.22996, 0.23689, 0.244, 0.25129, 0.25876, 
		0.26641, 0.27424, 0.28225, 0.29044, 0.29881, 0.30736, 0.31609, 
		0.325, 0.33409, 0.34336, 0.35281, 0.36244, 0.37225, 0.38224, 
		0.39241, 0.40276, 0.41329, 0.424, 0.43489, 0.44596, 0.45721, 
		0.46864, 0.48025, 0.49204, 0.50401, 0.51616, 
		0.52849, 0.541, 0.55369, 0.56656, 0.57961, 0.59284, 0.60625, 0.61984, 
		0.63361, 0.64756, 0.66169, 0.676, 0.69049, 0.70516, 0.72001, 0.73504, 0.75025, 0.76564, 
		0.78121, 0.79696, 0.81289, 0.829, 0.84529, 0.86176, 0.87841, 0.89524, 0.91225, 
    0.92944, 0.94681, 0.96436, 0.98209, 1.0, 1.0};

  private double driveLookup[];

  private double turnLookup[];

  private double MAX;

  private BasicPosKalman kalman;

  private AHRS gyro;

  public DriveSubsystem() {
    io = IO.getInstance();

    //Assigns the robot IDs from the robotMap.properties file
    rightMaster = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightMaster")), MotorType.kBrushless);
    rightSlave = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("rightSlave")), MotorType.kBrushless);
    leftMaster = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("leftMaster")), MotorType.kBrushless);
    leftSlave = new CANSparkMax(Integer.parseInt(Robot.m_robotMap.getValue("leftSlave")), MotorType.kBrushless);

    //set slaves to follow master motor controllers
    rightSlave.follow(rightMaster);
    leftSlave.follow(leftMaster);

    rightMaster.setInverted(false);
    rightSlave.setInverted(false);

    leftMaster.setInverted(false);
    leftSlave.setInverted(false);

    LookupGenerator driveGenerator = new LookupGenerator(Double.parseDouble(Robot.m_values.getValue("driveDeadband")), Double.parseDouble(Robot.m_values.getValue("driveMinPower")));
    LookupGenerator turnGenerator = new LookupGenerator(Double.parseDouble(Robot.m_values.getValue("turnDeadband")), Double.parseDouble(Robot.m_values.getValue("turnMinPower")),
                                                        Double.parseDouble(Robot.m_values.getValue("turnLowCutoff")), Double.parseDouble(Robot.m_values.getValue("turnMidPower")));

    driveGenerator.calcLookup(LookupType.QUADRATIC);
    driveLookup = driveGenerator.getLookupTable();

    turnGenerator.calcLookup(LookupType.DOUBLE_LINEAR);
    turnLookup = turnGenerator.getLookupTable();

    MAX = 1.0;

    Array2DRowRealMatrix init = new Array2DRowRealMatrix(new double[][] {{0}, {0}, {0}, {0}, {0}, {0}});
    Array2DRowRealMatrix initErr = new Array2DRowRealMatrix(new double[][] {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.1, 0.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.1, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.1, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.1}});

    kalman = new BasicPosKalman(init, initErr);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.zeroYaw();
  }

  //class convenience method to move the robot to save space in the different drive methods
  public void move(double r, double l) {

    //defensive code to prevent the values being passed to move from exceeding the accepted ranges on the motor controllers
    r = (r > MAX) ? MAX :  r;
    r = (r < -(MAX)) ? -(MAX) : r;
    l = (l > MAX) ? MAX : l;
    l = (l < -(MAX)) ? -(MAX) : l;

    //set motor powers, slaves do not need to be called as they were set to follow the master in the class constructor
    rightMaster.set(r);
    leftMaster.set(l);
  }

  //method to be called from the arcade drive command
  public void arcadeDrive(double x, double y) {
    // double r, l;

    // //set the values of r and l based off of x and y axes - may need to switch addition and subtraction
    // r = x - y;
    // l = x + y;

    // move(r, l);

    x = Utils.deadband(x, 0.05);
    y = Utils.deadband(y, 0.05);

    double difV = y - v; 
    double maxDifV = SmartDashboard.getNumber("maxAccel", 0.02d);

    if(difV > 0)
      v += (difV > maxDifV) ? maxDifV : difV;
    else
      v -= (Math.abs(difV) > maxDifV) ? maxDifV : Math.abs(difV);

    double s = (v < 0.1) ? SmartDashboard.getNumber("scale", 0.5d) * x * SmartDashboard.getNumber("zeroTurn", 0.5d) : SmartDashboard.getNumber("scale", 0.5) * x * v;

    double l = v - s;
    double r = v + s;

    avgAcc = ((l + r) / 2.0) * 14.08;

    move(r, l);
  }

  public void arcadeDriveLookup(double x, double y) {
    double r, l;

    //"I have no clue how this works ask Nathan" - Owen
    r = ((x > 0) ? turnLookup[(int) Math.floor(Math.abs(x) * 100)] : -turnLookup[(int) Math.floor(Math.abs(x) * 100)]) + ((y > 0) ? driveLookup[(int) Math.floor(Math.abs(y) * 100)] : -driveLookup[(int) Math.floor(Math.abs(y) * 100)]);
    l = ((x > 0) ? turnLookup[(int) Math.floor(Math.abs(x) * 100)] : -turnLookup[(int) Math.floor(Math.abs(x) * 100)]) - ((y > 0) ? driveLookup[(int) Math.floor(Math.abs(y) * 100)] : -driveLookup[(int) Math.floor(Math.abs(y) * 100)]);
    // SmartDashboard.putNumber("rightPower", r);
    // SmartDashboard.putNumber("leftPower", l);
    move(r, l);
  }

  //method to be called in the event of a tank drive
  public void tankDrive(double r, double l) {
    //this is pretty self explanatory
    move(r, l);
  }

  public void tankDriveLookup(double r, double l) {
    r = ((r > 0)) ? lookup[(int) Math.floor(Math.abs(r) * 100)] : -lookup[(int) Math.floor(Math.abs(r) * 100)];
    l = ((l > 0)) ? lookup[(int) Math.floor(Math.abs(l) * 100)] : -lookup[(int) Math.floor(Math.abs(l) * 100)];
    
    move(r, l);
  }

  @Override
  public void periodic() {
    arcadeDrive(io.getDriverController().getRightStickX(), io.getDriverController().getLeftStickY());

    kalman.predict();

    double r = rightMaster.getEncoder().getVelocity() * (Math.PI/120.0);
    double l = leftMaster.getEncoder().getVelocity() * (Math.PI/120.0);

    double theta = gyro.getYaw() * (180.0 / Math.PI);
    if(theta < 0.0) theta += (2.0 * Math.PI);

    double v = (r + l) / 2.0;
    double vx = v * Math.cos(theta);
    double vy = v * Math.sin(theta);
    double ay = avgAcc * Math.cos(theta);
    double ax = avgAcc * Math.sin(theta);
    
    Array2DRowRealMatrix xm = new Array2DRowRealMatrix(new double[][] {{0.0}, {0.0}, {vx}, {vy}, {ax}, {ay}}); // 3.14 / 120
    Array2DRowRealMatrix err = new Array2DRowRealMatrix(new double[][] {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.05, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.05, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.075, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.075}});
    kalman.measure(xm, err);
    kalman.update();

    double[][] result = kalman.getX().getData();

    double x = result[0][0];
    double y = result[1][0];
    vx = result[2][0];
    vy = result[3][0];
    ax = result[4][0];
    ay = result[5][0];

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("vx", vx);
    SmartDashboard.putNumber("vy", vy);
    SmartDashboard.putNumber("ax", ax);
    SmartDashboard.putNumber("ay", ay);
  }

  //I (Nathan) think this in miles/hr but I don't exactly remember
  //remember to document your code, kids
  public double getRightSpeed() {
    return rightMaster.getEncoder().getVelocity() * 0.001667;
  }

  public double getLeftSpeed() {
    return leftMaster.getEncoder().getVelocity() * 0.001667;
  }

}
