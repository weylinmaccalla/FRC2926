/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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


public class BooferSpoofer extends SubsystemBase {
  private final DoubleSolenoid WheelPiston = new DoubleSolenoid(Constants.SpooferForwardSol, Constants.SpooferReverseSol);
  private final WPI_VictorSPX SpooferMotor = new WPI_VictorSPX(Constants.SpooferMotor);

  public void DeployWheel(){
    WheelPiston.set(Value.kForward);
    SpooferMotor.set(.5);
  }
    
  public void DetractWheel(){
    WheelPiston.set(Value.kReverse);
    SpooferMotor.set(0);
  }
 
}
