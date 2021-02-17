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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.apache.commons.math3.linear.*;

import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase implements Subsystem {
  /**
   * Creates a new DriveSubsystem.
   */

  private double v = 0.0;

  private double vFt = 0.0;

  private double avgAcc = 0.0;

  private double prevL = 0.0;
  private double prevR = 0.0;
  
  public CANSparkMax rightMaster;
  public CANSparkMax rightSlave;
  public CANSparkMax leftMaster;
  public CANSparkMax leftSlave;

  private IO io;

  private BasicPosKalman kalman;

  private AHRS gyro;

  // value that accounts for the tire's interactions with the tile floor
  private final double SURFACE_SCALE_FACTOR;

  // conversion factor from RPM returned from encoders to ft/s
  private final double RPM_TO_FTPS;
  // conversion factor from RPM returned from encoders to mi/h
  private final double RPM_TO_MPH;
  // absolute maximum output power of the drive, from 0.0 to 1.0
  private final double MAX;

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

    rightMaster.setInverted(true);
    rightSlave.setInverted(true);

    leftMaster.setInverted(false);
    leftSlave.setInverted(false);

    SURFACE_SCALE_FACTOR = Double.parseDouble(Robot.m_values.getValue("tileScaleFactor"));

    RPM_TO_FTPS = Double.parseDouble(Robot.m_values.getValue("rpmToFeetPerSecond"));
    RPM_TO_MPH = Double.parseDouble(Robot.m_values.getValue("rpmToMilesPerHour"));

    MAX = Double.parseDouble(Robot.m_values.getValue("maxOutput"));

    RealMatrix init = new Array2DRowRealMatrix(new double[][] {{0}, {0}, {0}, {0}, {0}, {0}});
    RealMatrix initErr = new Array2DRowRealMatrix(new double[][] {{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.01, 0.0, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.2, 0.0, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.2, 0.0, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
                                                                            {0.0, 0.0, 0.0, 0.0, 0.0, 0.2}});

    kalman = new BasicPosKalman(init, initErr);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

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

    x = Utils.deadband(x, 0.05);
    y = Utils.deadband(y, 0.05);

    // difference between current velocity and commanded velocity in the y direction
    double difV = y - v; 
    SmartDashboard.putNumber("difV", difV);
    double maxDifV = SmartDashboard.getNumber("maxAccel", 0.02d);

    if(difV > 0) {
      v += (difV > maxDifV) ? maxDifV : difV;
    } else {
      v -= (Math.abs(difV) > maxDifV) ? maxDifV : Math.abs(difV);
    }

    double s = (Math.abs(v) < 0.1) ? SmartDashboard.getNumber("scale", 0.5d) * x * SmartDashboard.getNumber("zeroTurn", 0.5d) : SmartDashboard.getNumber("scale", 0.5) * x * v;


    double l = v - s;
    double r = v + s;


    move(r, l);
  }

  //method to be called in the event of a tank drive
  public void tankDrive(double r, double l) {
    //this is pretty self explanatory
    move(r, l);
  }

  //I (Nathan) think this in miles/hr but I don't exactly remember
  //remember to document your code, kids
  public double getRightSpeed() {
    return rightMaster.getEncoder().getVelocity() * RPM_TO_MPH;
  }

  public double getLeftSpeed() {
    return leftMaster.getEncoder().getVelocity() * RPM_TO_MPH;
  }

  private void updateKalman() {
    double[][] result = kalman.getX().getData();

    kalman.predict();

    // the 0.89 is to compensate for the wheel against the surface that it is driving on, may need to be changed
    // for different surfaces
    double r = rightMaster.getEncoder().getVelocity() * RPM_TO_FTPS * SURFACE_SCALE_FACTOR; // (Math.PI / 120.0);
    double l = -leftMaster.getEncoder().getVelocity() * RPM_TO_FTPS * SURFACE_SCALE_FACTOR; // (Math.PI / 120.0);

    // calculate the average velocity in ft/s between the two wheels
    vFt = (r + l) / 2.0;

    // calculate the average acceleration in ft/s/s between the two wheels
    avgAcc = ((r - prevR) + (l - prevL)) / 0.04;

    // push raw gyro data to the dashboard for debugging
    SmartDashboard.putNumber("rawAngle", gyro.getAngle());

    // get the gyro angle in degrees and convert to radians since that's what Java's trig functions use
    double theta = (gyro.getAngle()) * (Math.PI / 180.0);
    SmartDashboard.putNumber("theta", theta);

    // get the current position values and update them with the current velocity and acceleration readings
    double x = result[0][0];
    double y = result[1][0];

    // break the velocity down into its components
    double vx = vFt * Math.cos(theta);
    double vy = vFt * Math.sin(theta);

    // do the same with acceleration
    double ax = avgAcc * Math.cos(theta);
    double ay = avgAcc * Math.sin(theta);

    // use dx = v0t + 1/2at^2 to find our new measured position
    x += (vx * 0.02) + (ax * 0.002);
    y += (vy * 0.02) + (ay * 0.002);
    
    // matrices to update the Kalman filter with our measurements
    RealMatrix xm = new Array2DRowRealMatrix(new double[][] {{x}, {y}, {vx}, {vy}, {ax}, {ay}}); // 3.14 / 120
    RealMatrix err = new Array2DRowRealMatrix(new double[][] {{0.16, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.16, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.05, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.05, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.07, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.07}});
    kalman.measure(xm, err);
    kalman.update();

    // update our previous left and right acceleration values
    prevL = l;
    prevR = r;

    // get the result from our Kalman filter
    result = kalman.getX().getData();

    // store result to variables, can add calls to push this data to driver station
    x = result[0][0];
    y = result[1][0];
    vx = result[2][0];
    vy = result[3][0];
    ax = result[4][0];
    ay = result[5][0];
  }

  // resets the kalman filter's values to 0, useful for getting rid of accumulated errors
  public void resetKalman() {
    RealMatrix init = new Array2DRowRealMatrix(new double[][] {{0}, {0}, {0}, {0}, {0}, {0}});
    RealMatrix initErr = new Array2DRowRealMatrix(new double[][] {{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.01, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.2, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.2, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.2}});

    kalman = new BasicPosKalman(init, initErr);
    gyro.reset();  
  }

  // same as the other definition for this method, but sets the starting position to (x, y)
  public void resetKalman(double x, double y) {
    RealMatrix init = new Array2DRowRealMatrix(new double[][] {{x}, {y}, {0}, {0}, {0}, {0}});
    RealMatrix initErr = new Array2DRowRealMatrix(new double[][] {{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.01, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.2, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.2, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.2}});

    kalman = new BasicPosKalman(init, initErr);
    gyro.reset();
  }

  @Override
  public void periodic() {
    if(io.getDriverController().isButtonPressed("A")) resetKalman();

    updateKalman();
  }

}
