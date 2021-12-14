// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.Joystick;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import edu.wpi.first.wpilibj.controller.PIDController;

public class Robot extends TimedRobot {
  PIDController pid = new PIDController(.5, 0, 0);
  private final VictorSP TurretMotor = new VictorSP(0);
  private final Joystick joy1 = new Joystick(0);
  // .5 for P Value
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  Encoder enc;

  private static final double cpr = 7 / 4; // if am-2861a

  private static final double whd = 1.5; // for 6 inch wheel

  @Override
  public void robotInit() {
    enc = new Encoder(0, 1);
    enc.setDistancePerPulse(((Math.PI * whd / cpr) / 17597.238550001526) * 360); // distance per pulse is pi* (wheel
                                                                                 // diameter / counts per revolution)
  }

  @Override
  public void robotPeriodic() {
    double dist = enc.getDistance();
    SmartDashboard.putNumber("Encoder", dist);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double OppositeAmp = joy1.getRawAxis(0); // Left and Right on Joystick
    double AdjacentAmp = joy1.getRawAxis(1); // Up and Down on Joystick
    double AngleRadians = Math.atan(Math.abs(OppositeAmp) / Math.abs(AdjacentAmp)); // Obtaining angle using Inverse
                                                                                    // Tan, using the joysticks to
                                                                                    // obtain side lengths
    double AngleDegrees = Math.toDegrees(AngleRadians); // Converting Radians to Degrees
    if (OppositeAmp < 0 && AdjacentAmp < 0) // Gives you angle in second quadrant
    {
      AngleDegrees = 360 - AngleDegrees;
    } else if (OppositeAmp < 0 && AdjacentAmp > 0) // Gives you angle in third quadrant
    {
      AngleDegrees += 180;
    } else if (OppositeAmp > 0 && AdjacentAmp > 0) // Gives you angle in fourth quadrant
    {
      AngleDegrees = 180 - AngleDegrees;
    }
    if (OppositeAmp == 0 && AdjacentAmp > 0) // Special case if joystick is down
    {
      AngleDegrees = 180;
    }
    if (OppositeAmp < 0 && AdjacentAmp == 0) // Special case if joystick is pointed to left
    {
      AngleDegrees = 270;
    }

    SmartDashboard.putNumber("Degrees", AngleDegrees);
    double MotorSpeed = pid.calculate(enc.getDistance(), AngleDegrees);
    SmartDashboard.putNumber("Motor Speed", MotorSpeed);

    TurretMotor.set(MotorSpeed);

    if (Math.abs(joy1.getRawAxis(5)) > .1) {
      TurretMotor.set(joy1.getRawAxis(5) * -1.0);
    }
    if (joy1.getRawButton(1)) {
      enc.reset();
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
