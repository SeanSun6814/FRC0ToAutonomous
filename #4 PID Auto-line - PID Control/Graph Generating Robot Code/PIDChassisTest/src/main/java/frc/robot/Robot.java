/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  private Spark leftMotor1 = new Spark(0);
  private Spark leftMotor2 = new Spark(1);
  private Spark rightMotor1 = new Spark(2);
  private Spark rightMotor2 = new Spark(3);

  private Joystick joy1 = new Joystick(0);
  private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);;
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  private Logger logger;

  @Override
  public void robotInit() {
    logger = new Logger();
  }

  @Override
  public void autonomousInit() {
    logger = new Logger();
    encoder.reset();
    errorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();
    logger.log(",,,,,,,,,");
    logger.log(kP);
    logger.log(kI);
    logger.log(kD);
    logger.log(iZone);
    logger.log(System.currentTimeMillis());
    logger.log(Timer.getFPGATimestamp());
    logger.log("________STARTING________");
    logger.log("\n");
    logger.log("Timer.getFPGATimestamp()");
    logger.log("setpoint");
    logger.log("sensorPosition");
    logger.log("error");
    logger.log("iZone");
    logger.log("errorSum");
    logger.log("errorRate");

    logger.log("kP * error");
    logger.log("kI * errorSum");
    logger.log("kD * errorRate");
    logger.log("outputSpeed");
    logger.log("\n");

  }

  final double kP = 0.5;
  final double kI = 0.5;
  final double kD = 0.1;
  final double iZone = 1;

  double setpoint = 0;
  double errorSum = 0;
  double lastError = 0;
  double lastTimestamp = 0;

  @Override
  public void autonomousPeriodic() {
    // get joystick command
    if (joy1.getRawButton(1)) {
      setpoint = 10;
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }

    // get sensor information
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;
    double timeInterval = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(error) < iZone) {
      errorSum += error * timeInterval;
    }

    double errorRate = (error - lastError) / timeInterval;

    double outputSpeed = kP * error + kI * errorSum + kD * errorRate;

    SmartDashboard.putNumber("PID output", outputSpeed);

    // output to motors
    leftMotor1.set(outputSpeed);
    leftMotor2.set(outputSpeed);
    rightMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);

    lastError = error;
    lastTimestamp = Timer.getFPGATimestamp();

    // -Logger-

    logger.log(Timer.getFPGATimestamp());
    logger.log(setpoint);
    logger.log(sensorPosition);
    logger.log(error);
    logger.log((Math.abs(error) < iZone) ? 1 : 0);
    logger.log(errorSum);
    logger.log(errorRate);

    logger.log(kP * error);
    logger.log(kI * errorSum);
    logger.log(kD * errorRate);
    logger.log(outputSpeed);
    logger.log("\n");
  }

  @Override
  public void disabledInit() {
    if (logger != null)
      logger.flush();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double speed = -joy1.getRawAxis(1) * 0.6;
    double turn = joy1.getRawAxis(4) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

    leftMotor1.set(left);
    leftMotor2.set(left);
    rightMotor1.set(-right);
    rightMotor2.set(-right);
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
  }

}

// ONLY P CONTROL=========================================================
// final double kP = 0.05;

// double setpoint = 0;

// @Override
// public void autonomousPeriodic() {
// // get joystick command
// if (joy1.getRawButton(1)) {
// setpoint = 20;
// } else if (joy1.getRawButton(2)) {
// setpoint = 0;
// }

// // get sensor information
// double sensorPosition = encoder.get();

// // calculations
// double error = setpoint - sensorPosition;
// double outputSpeed = kP * error;

// // output to motors
// leftMotor1.set(outputSpeed);
// leftMotor2.set(outputSpeed);
// rightMotor1.set(-outputSpeed);
// rightMotor2.set(-outputSpeed);
// }

//