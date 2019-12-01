/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
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

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
    encoder.reset();
  }

  final double kP = 0.5;

  double setpoint = 0;

  @Override
  public void autonomousPeriodic() {
    // get joystick command
    if (joy1.getRawButton(1)) {
      setpoint = 10;
    } else if (joy1.getRawButton(2)) {
      setpoint = 0;
    }

    // get sensor position
    double sensorPosition = encoder.get() * kDriveTick2Feet;

    // calculations
    double error = setpoint - sensorPosition;

    double outputSpeed = kP * error;

    // output to motors
    leftMotor1.set(outputSpeed);
    leftMotor2.set(outputSpeed);
    rightMotor1.set(-outputSpeed);
    rightMotor2.set(-outputSpeed);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
