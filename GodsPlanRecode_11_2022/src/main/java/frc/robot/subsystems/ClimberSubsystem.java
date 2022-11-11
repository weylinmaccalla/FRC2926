// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  // Initializes a DigitalInput on DIO 0
  DigitalInput RightLimitSwitch = new DigitalInput(Constants.ClimberSubsystemConstants.RightClimberMotor_CanID);
  DigitalInput LeftLimitSwitch = new DigitalInput(Constants.ClimberSubsystemConstants.LeftClimberMotor_CanID);

  // Assigns Motors left & right with a CANSparkMax and constructors
  CANSparkMax LeftClimberMotor = new CANSparkMax(Constants.ClimberSubsystemConstants.LeftClimberMotor_CanID,MotorType.kBrushless);
  CANSparkMax RightClimberMotor = new CANSparkMax(Constants.ClimberSubsystemConstants.RightClimberMotor_CanID,MotorType.kBrushless);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  }

  public void LowerClimber() {
    if (RightLimitSwitch.get()) { 
      RightClimberMotor.set(-0.5);
    }

    if (LeftLimitSwitch.get()) { 
      LeftClimberMotor.set(-0.5);
    }

  }

  public void RaiseClimber() {
    RightClimberMotor.set(0.5);
    LeftClimberMotor.set(0.5);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}