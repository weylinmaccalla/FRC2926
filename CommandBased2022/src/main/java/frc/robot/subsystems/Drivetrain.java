// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(Constants.LEFT_MOTOR_1_CAN_ID);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(Constants.LEFT_MOTOR_2_CAN_ID);

  SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);

  private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(Constants.RIGHT_MOTOR_1_CAN_ID);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(Constants.RIGHT_MOTOR_2_CAN_ID);
  SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);

  DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);


  public Drivetrain() {}

  public void curvatureDrive (Joystick joy1){
    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double speed = forward - reverse;
    double turn = joy1.getRawAxis(0);
    double adjustedSpeed = Math.pow(speed, 3);

    if (reverse > 0) {
      turn = turn * -1;
    }

    if (joy1.getRawButton(5)) {
      drivetrain.curvatureDrive(adjustedSpeed, -.4, true);
    }

    if (joy1.getRawButton(6)) {
      drivetrain.curvatureDrive(adjustedSpeed, .4, true);
    }

    if ((joy1.getRawButton(5) == false) && (joy1.getRawButton(6) == false)) {
      drivetrain.curvatureDrive(adjustedSpeed, turn, false);
    }

    drivetrain.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
