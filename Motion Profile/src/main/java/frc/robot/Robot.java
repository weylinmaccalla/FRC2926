/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;



public class Robot extends TimedRobot {
  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  Compressor c;
  DoubleSolenoid Piston;
  private final WPI_TalonSRX LeftSlave = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMaster = new WPI_TalonSRX(2);
 
  private final WPI_TalonSRX RightSlave = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMaster = new WPI_TalonSRX(4);
  
   
  private final Joystick joy1 = new Joystick(0);


  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    
    LeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0 , 30);
    RightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0 , 30);

    LeftMaster.configNominalOutputForward(0, 30);
		LeftMaster.configNominalOutputReverse(0, 30);
		LeftMaster.configPeakOutputForward(1, 30);
    LeftMaster.configPeakOutputReverse(-1, 30);

    RightMaster.configNominalOutputForward(0, 30);
		RightMaster.configNominalOutputReverse(0, 30);
		RightMaster.configPeakOutputForward(1, 30);
    RightMaster.configPeakOutputReverse(-1, 30);
    
    LeftMaster.configNominalOutputForward(0, 30);
		LeftMaster.configNominalOutputReverse(0, 30);
		LeftMaster.configPeakOutputForward(1, 30);
		LeftMaster.configPeakOutputReverse(-1, 30);

    RightMaster.config_kF(0, 1.00294118 ,30);
		RightMaster.config_kP(0, 1 , 30);
		RightMaster.config_kI(0, 0.00749993324 , 30);
    RightMaster.config_kD(0, 0, 30);

    LeftMaster.config_kF(0, 1.00294118 ,30);
		LeftMaster.config_kP(0, 1 , 30);
		LeftMaster.config_kI(0, 0.00749993324 , 30);
		LeftMaster.config_kD(0, 0, 30);

    Piston = new DoubleSolenoid(0, 1);
    c = new Compressor(0);
    c.setClosedLoopControl(true);
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
    
    SmartDashboard.putBoolean("Under Max Pressure", c.getPressureSwitchValue());
    SmartDashboard.putNumber("Total Power Consumption", pdp.getTotalCurrent());
    SmartDashboard.putNumber("Right Motor 1 Amps", pdp.getCurrent(1));
    SmartDashboard.putNumber("Right Motor 2 Amps", pdp.getCurrent(2));
    SmartDashboard.putNumber("Left Motor 1 Amps", pdp.getCurrent(14));
    SmartDashboard.putNumber("Left Motor 2 Amps", pdp.getCurrent(15));

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
 
    SmartDashboard.putBoolean("Low Pressure", c.getPressureSwitchValue());
    SmartDashboard.putNumber("Total Power Consumption", pdp.getTotalCurrent());

  }


  @Override
  public void teleopPeriodic() {
    
    RightUltimateMaster.set(ControlMode.Velocity, RightMotorSpeed);
    LeftMaster.follow(RightMaster, FollowerType.AuxOutput1);
    LeftSlave.follow(RightMaster;
    RightSlave.follow(ControlMode.Follower,4);

    RightMaster.setInverted(true);
    RightSlave.setInverted(true);
    LeftMaster.setInverted(false);
    LeftMaster.setInverted(false);
    RightMaster.setSafetyEnabled(false);
    RightSlave.setSafetyEnabled(false);
    LeftMaster.setSafetyEnabled(false);
    LeftSlave.setSafetyEnabled(false);
    double Forward = joy1.getRawAxis(3);
    double Reverse = joy1.getRawAxis(2);
    double Turn= joy1.getRawAxis(0);
    double Speed = (Forward-Reverse)*1000;
    if (Math.abs(Turn) < 0.15) {
      Turn = 0;
  }
    double RightMotorSpeed= Speed-(Turn*1000);
    double LeftMotorSpeed= Speed+(Turn*1000);  
    
    RightMaster.setSensorPhase(true);
    LeftMaster.setSensorPhase(true);
    RightSlave.setSensorPhase(true);
    LeftSlave.setSensorPhase(true);
    RightMaster.getFaults(_rightfaults);
    LeftMaster.getFaults(_leftfaults);
    SmartDashboard.putNumber("Right Sensor Vel:", RightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Sensor Vel:", LeftMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Sensor Pos:", RightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("LeftSensor Pos:", LeftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Out %", RightMaster.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Out %", LeftMaster.getMotorOutputPercent());
    SmartDashboard.putBoolean("Left Out Of Phase:", _leftfaults.SensorOutOfPhase);
    SmartDashboard.putBoolean("Right Out Of Phase:", _rightfaults.SensorOutOfPhase);
    SmartDashboard.putBoolean("Under Max Pressure", c.getPressureSwitchValue());
    SmartDashboard.putNumber("Total Power Consumption", pdp.getTotalCurrent());
    SmartDashboard.putNumber("Right Motor 1 Amps", pdp.getCurrent(12));
    SmartDashboard.putNumber("Right Motor 2 Amps", pdp.getCurrent(13));
    SmartDashboard.putNumber("Left Motor 1 Amps", pdp.getCurrent(14));
    SmartDashboard.putNumber("Left Motor 2 Amps", pdp.getCurrent(15));

  }
 
  @Override
  public void testPeriodic() {
  }
}

