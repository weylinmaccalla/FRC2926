// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
CANSparkMax intakemotor = new CANSparkMax(Constants.IntakeSubsystemConstants.intake_CanID, MotorType.kBrushless);
DoubleSolenoid intakesolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeSubsystemConstants.Right_Solenoid, Constants.IntakeSubsystemConstants.Lift_Solenoid);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void RaiseIntake(){
  intakemotor.set(0);
  intakesolenoid.set(DoubleSolenoid.Value.kForward);

  } 

  public void LowerIntake(){
    intakemotor.set(.5);
    intakesolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}
