/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;



public class Robot extends TimedRobot {
  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  AHRS ahrs;
  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  Compressor c;
  DoubleSolenoid Piston;
  private final WPI_TalonSRX LeftSlave = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMaster = new WPI_TalonSRX(2);
 
  private final WPI_TalonSRX RightSlave = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMaster = new WPI_TalonSRX(4);
  
   
  private final Joystick joy1 = new Joystick(0);

  CameraServer server;

  
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

    CameraServer.getInstance().startAutomaticCapture();

    Piston = new DoubleSolenoid(0, 1);
    c = new Compressor(0);
    c.setClosedLoopControl(true);
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      ahrs.enableLogging(true);
    } catch (final RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

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
    SmartDashboard.putBoolean("NavX Connected?", ahrs.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating?", ahrs.isCalibrating());
    SmartDashboard.putNumber("Compass Heading", ahrs.getCompassHeading());
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
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());
    SmartDashboard.putBoolean("Low Pressure", c.getPressureSwitchValue());
    SmartDashboard.putNumber("Total Power Consumption", pdp.getTotalCurrent());

  }


  @Override
  public void teleopPeriodic() {
    RightMaster.setInverted(true);
    RightSlave.setInverted(true);
    LeftMaster.setInverted(false);
    LeftMaster.setInverted(false);
    RightMaster.setSafetyEnabled(false);
    RightSlave.setSafetyEnabled(false);
    LeftMaster.setSafetyEnabled(false);
    LeftSlave.setSafetyEnabled(false);
  }
    double mQuickStopAccumulator = 0;
    public static final double kThrottleDeadband = 0.02;
    private static final double kWheelDeadband = 0.02;
    private static final double kTurnSensitivity = 1.0;
    double Forward = joy1.getRawAxis(3);
    double Reverse = joy1.getRawAxis(2);
    double wheel= joy1.getRawAxis(0);
    double throttle = Forward-Reverse;
    double overPower;
    double angularPower;
    boolean isQuickTurn = false;
    {}
    
    if (isQuickTurn) {
      if (Math.abs(throttle) < 0.2) {
          double alpha = 0.1;
          mQuickStopAccumulator = (1 - alpha) * mQuickStopAccumulator + alpha * Util.limit(wheel, 1.0) * 2;
      }
      overPower = 1.0;
      angularPower = wheel;
  } else {
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * kTurnSensitivity - mQuickStopAccumulator;
      if (mQuickStopAccumulator > 1) {
          mQuickStopAccumulator -= 1;
      } else if (mQuickStopAccumulator < -1) {
          mQuickStopAccumulator += 1;
      } else {
          mQuickStopAccumulator = 0.0;
      }
  }
  

  double RightSpeed = throttle - angularPower;
  double LeftSpeed = throttle + angularPower;
  if (LeftSpeed > 1.0) {
      RightSpeed -= overPower * (LeftSpeed - 1.0);
      LeftSpeed = 1.0;
  } else if (rightPwm > 1.0) {
      LeftSpeed -= overPower * (RightSpeed - 1.0);
      RightSpeed = 1.0;
  } else if (leftPwm < -1.0) {
      RightSpeed += overPower * (-1.0 - LeftSpeed);
      LeftSpeed = -1.0;
  } else if (rightPwm < -1.0) {
      LeftSpeed += overPower * (-1.0 - RightSpeed);
      RightSpeed = -1.0;
  }
  mSignal.rightMotor = rightPwm;
  mSignal.leftMotor = leftPwm;
  return mSignal;
  }
  
    RightMaster.set(ControlMode.Velocity, RightMotorSpeed);
    LeftMaster.set(ControlMode.Velocity, LeftMotorSpeed);
    LeftSlave.set(ControlMode.Follower,2);
    RightSlave.set(ControlMode.Follower,4);
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
    SmartDashboard.putBoolean("NavX Connected?", ahrs.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating?", ahrs.isCalibrating());
    SmartDashboard.putNumber("Compass Heading", ahrs.getCompassHeading());
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
