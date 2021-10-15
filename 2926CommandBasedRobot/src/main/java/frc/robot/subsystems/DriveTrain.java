/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
 
   private final WPI_TalonSRX LeftSlave = new WPI_TalonSRX(Constants.LeftMotor1);
   private final WPI_TalonSRX LeftMaster = new WPI_TalonSRX(Constants.LeftMotor2);
  
   private final WPI_TalonSRX RightSlave = new WPI_TalonSRX(Constants.RightMotor1);
   private final WPI_TalonSRX RightMaster = new WPI_TalonSRX(Constants.RightMotor2);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
