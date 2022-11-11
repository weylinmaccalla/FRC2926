// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax rightMotor1 = new CANSparkMax(Constants.DriveSubsystemConstants.rightMotor1_CanID, MotorType.kBrushless);
  CANSparkMax rightMotor2 = new CANSparkMax(Constants.DriveSubsystemConstants.rightMotor2_CanID, MotorType.kBrushless);
  CANSparkMax leftMotor1 = new CANSparkMax(Constants.DriveSubsystemConstants.leftMotor1_CanID, MotorType.kBrushless);
  CANSparkMax leftMotor2 = new CANSparkMax(Constants.DriveSubsystemConstants.leftMotor2_CanID, MotorType.kBrushless);

  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // TODO: Invert motors as needed. 
    rightMotor1.setInverted(false);
    rightMotor2.setInverted(false);
    leftMotor1.setInverted(false);
    leftMotor2.setInverted(false);
  }

  public void arcadeDrive(double speed, double rotation)
  {
    drive.arcadeDrive(speed, rotation);
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
