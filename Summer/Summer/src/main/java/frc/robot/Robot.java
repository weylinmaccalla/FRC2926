/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class Robot extends TimedRobot {
  private final Joystick joy1 = new Joystick(0);
  private final Servo XAxis = new Servo(0);
  private final Servo YAxis = new Servo(1);
  private final PWM Red = new PWM(2);
  private final PWM Blue = new PWM(3);
  private final PWM Green = new PWM(4);
  
  @Override
  public void robotInit() {
 
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
 
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopPeriodic() {
 
     XAxis.set(0.5 + ((joy1.getRawAxis(0))/2));
    SmartDashboard.putNumber("X", joy1.getRawAxis(0));
    YAxis.set(0.5 + ((joy1.getRawAxis(1))/2));
    SmartDashboard.putNumber("Y", joy1.getRawAxis(1));
    Red.setRaw(0);
    Blue.setRaw(235);
    Green.setRaw(230);

    

  }

  @Override
  public void testPeriodic() {
  }
}
