/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallHopper extends SubsystemBase {
  private final DoubleSolenoid EntranceHatch = new DoubleSolenoid(Constants.EntranceForwardSol, Constants.EntranceReverseSol);
  private final DoubleSolenoid ExitHatch = new DoubleSolenoid(Constants.ExitForwardSol, Constants.ExitReverseSol);
  private final WPI_VictorSPX HopperMotor = new WPI_VictorSPX(Constants.HopperMotor);
  
  public BallHopper() { 
    HopperMotor.setInverted(false); 
  }
 public void Neutral(){
   HopperMotor.set(0);
   EntranceHatch.set(Value.kReverse);
   ExitHatch.set(Value.kForward);
 }
 public void CollectCells(){
   HopperMotor.set(.25);
   EntranceHatch.set(Value.kReverse);
 }

 public void DropCells(){
   HopperMotor.set(.25);
   ExitHatch.set(Value.kReverse);
 }
}

