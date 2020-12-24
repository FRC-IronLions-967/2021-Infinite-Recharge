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
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.utils.Utils;
import frc.robot.utils.kalman.BasicPosKalman;
import frc.robot.values.LookupGenerator;
import frc.robot.values.LookupType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

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
  
  private CANSparkMax rightMaster;
  private CANSparkMax rightSlave;
  private CANSparkMax leftMaster;
  private CANSparkMax leftSlave;

  private CANPIDController rightController;
  private CANPIDController leftController;

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

  private double angleOffset = 0.0;

  // value that accounts for the tire's interactions with the tile floor
  private final double SURFACE_SCALE_FACTOR = 0.89;

  // conversion factor from RPM returned from encoders to ft/s
  private final double RPM_TO_FTPS = 0.002445;
  // conversion factor from RPM returned from encoders to mi/h
  private final double RPM_TO_MPH = 0.001667;
  // conversion factor from revolutions returned from encoders to feet
  private final double REV_TO_FT = 0.1467;

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

    leftMaster.setInverted(true);
    leftSlave.setInverted(true);

    rightController = rightMaster.getPIDController();
    // these are all completely made up and need to be tuned
    rightController.setP(0.5);
    rightController.setI(0.0);
    rightController.setD(0.0);
    rightController.setOutputRange(-0.5, 0.5);
    rightMaster.setClosedLoopRampRate(1.0);
    rightSlave.setClosedLoopRampRate(1.0);
    rightMaster.getEncoder().setPositionConversionFactor(1.0);
    rightSlave.getEncoder().setPositionConversionFactor(1.0);

    leftController = leftMaster.getPIDController();
    leftController.setP(0.5);
    leftController.setI(0.0);
    leftController.setD(0.0);
    leftController.setOutputRange(-0.5, 0.5);
    leftMaster.setClosedLoopRampRate(1.0);
    leftSlave.setClosedLoopRampRate(1.0);
    leftMaster.getEncoder().setPositionConversionFactor(1.0);
    leftSlave.getEncoder().setPositionConversionFactor(1.0);

    LookupGenerator driveGenerator = new LookupGenerator(Double.parseDouble(Robot.m_values.getValue("driveDeadband")), Double.parseDouble(Robot.m_values.getValue("driveMinPower")));
    LookupGenerator turnGenerator = new LookupGenerator(Double.parseDouble(Robot.m_values.getValue("turnDeadband")), Double.parseDouble(Robot.m_values.getValue("turnMinPower")),
                                                        Double.parseDouble(Robot.m_values.getValue("turnLowCutoff")), Double.parseDouble(Robot.m_values.getValue("turnMidPower")));

    driveGenerator.calcLookup(LookupType.QUADRATIC);
    driveLookup = driveGenerator.getLookupTable();

    turnGenerator.calcLookup(LookupType.DOUBLE_LINEAR);
    turnLookup = turnGenerator.getLookupTable();

    MAX = 1.0;

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

  public void movePID(double r, double l, ControlType type) {
    rightController.setReference(r, type);
    leftController.setReference(l, type);
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

  public void arcadeDriveLookup(double x, double y) {
    double r, l;

    //"I have no clue how this works ask Nathan" - Owen
    r = ((x > 0) ? turnLookup[(int) Math.floor(Math.abs(x) * 100)] : -turnLookup[(int) Math.floor(Math.abs(x) * 100)]) + ((y > 0) ? driveLookup[(int) Math.floor(Math.abs(y) * 100)] : -driveLookup[(int) Math.floor(Math.abs(y) * 100)]);
    l = ((x > 0) ? turnLookup[(int) Math.floor(Math.abs(x) * 100)] : -turnLookup[(int) Math.floor(Math.abs(x) * 100)]) - ((y > 0) ? driveLookup[(int) Math.floor(Math.abs(y) * 100)] : -driveLookup[(int) Math.floor(Math.abs(y) * 100)]);
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

  //I (Nathan) think this in miles/hr but I don't exactly remember
  //remember to document your code, kids
  public double getRightSpeed() {
    return rightMaster.getEncoder().getVelocity() * RPM_TO_MPH;
  }

  public double getLeftSpeed() {
    return leftMaster.getEncoder().getVelocity() * RPM_TO_MPH;
  }

  public double getRightCurrent() {
    return rightMaster.getOutputCurrent();
  }

  public double getLeftCurrent() {
    return leftMaster.getOutputCurrent();
  }

  private void updateKalman() {
    double[][] result = kalman.getX().getData();

    kalman.predict();

    // the 0.89 is to compensate for the wheel against the surface that it is driving on, may need to be changed
    // for different surfaces
    double r = rightMaster.getEncoder().getVelocity() * RPM_TO_FTPS * SURFACE_SCALE_FACTOR; // (Math.PI / 120.0);
    double l = -leftMaster.getEncoder().getVelocity() * RPM_TO_FTPS * SURFACE_SCALE_FACTOR; // (Math.PI / 120.0);
    vFt = (r + l) / 2.0;
    avgAcc = ((r - prevR) + (l - prevL)) / 0.04;

    SmartDashboard.putNumber("rawAngle", gyro.getAngle() - angleOffset);
    double theta = (gyro.getAngle() - angleOffset) * (Math.PI / 180.0);
    SmartDashboard.putNumber("theta", theta);

    // get the current position values and update them with the current velocity and acceleration readings
    double x = result[0][0];
    double y = result[1][0];

    double vx = vFt * Math.cos(theta);
    double vy = vFt * Math.sin(theta);
    double ax = avgAcc * Math.cos(theta);
    double ay = avgAcc * Math.sin(theta);

    x += (vx * 0.02) + (ax * 0.002);
    y += (vy * 0.02) + (ay * 0.002);
    
    RealMatrix xm = new Array2DRowRealMatrix(new double[][] {{x}, {y}, {vx}, {vy}, {ax}, {ay}}); // 3.14 / 120
    RealMatrix err = new Array2DRowRealMatrix(new double[][] {{0.16, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.16, 0.0, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.05, 0.0, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.05, 0.0, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.07, 0.0},
                                                                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.07}});
    kalman.measure(xm, err);
    kalman.update();

    prevL = l;
    prevR = r;

    result = kalman.getX().getData();

    x = result[0][0];
    y = result[1][0];
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

  // resets the kalman filter's values to 0, useful for getting rid of accumulated errors
  public void resetKalman() {
    RealMatrix init = new Array2DRowRealMatrix(new double[][] {{0}, {0}, {0}, {0}, {0}, {0}});
    RealMatrix initErr = new Array2DRowRealMatrix(new double[][] {{0.01, 0.0, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.01, 0.0, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.2, 0.0, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.2, 0.0, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.2, 0.0},
                                                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.2}});
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
  }

  public double[] getKalmanData() {
    return kalman.getX().getData()[0];
  }

  // turns to the angle specified (in radian, absolute)
  // the values here need to be tuned, Im guessing at them right now
  public void turnToAngle(double angle) {
    System.out.println("Inside turnToAngle");
    // proprotional constant to multiply p by
    double p = 1.0e-3;
    // margin of error in revolutions
    double moe = 0.02;
    double theta = gyro.getAngle() * (Math.PI/180.0);
    double error = (theta - angle) / (2.0 * Math.PI);;

    while(Math.abs(error) > moe) {
      // if error is positive, this will cause the robot to rotate clockwise, decreasing the angle
      // if error is negative, this will cause the robot to rotate counterclockwise, increasing the angle
      System.out.println("Error: " + error);
      System.out.println("Angle: " + angle);
      System.out.println("Theta: " + theta);
      rightMaster.set(-p * error);
      leftMaster.set(p * error);

      theta = gyro.getAngle() * (Math.PI/180.0);
      error = (theta - angle) / (2.0 * Math.PI);
    }

  }

  // drives the specified distance in feet
  public void driveDistance(double distance) {
    System.out.println("Inside driveDistance");
    distance *= REV_TO_FT;
    rightController.setReference(distance, ControlType.kPosition);
    leftController.setReference(distance, ControlType.kPosition);
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    if(io.getDriverController().isButtonPressed("A")) resetKalman();

    updateKalman();
  }

}
