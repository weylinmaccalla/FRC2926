// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 color = new ColorSensorV3(i2cPort);

  CANSparkMax brush = new CANSparkMax(Constants.ShooterSubsystemConstants.brush_canID, MotorType.kBrushed);
  CANSparkMax lancher = new CANSparkMax(Constants.ShooterSubsystemConstants.lancher_canID, MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void shoot() {
    lancher.set(.5);
    brush.set(.3); 
  }

  public void stopshoot() {
    lancher.set(0);
    brush.set(0);
  }
}