/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;



public class Robot extends TimedRobot {
  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  AHRS ahrs;

  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  Compressor c;
  DoubleSolenoid DeploySpinner;

  private final WPI_TalonSRX LeftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMotor2 = new WPI_TalonSRX(2);

  SpeedControllerGroup leftmotors = new SpeedControllerGroup(LeftMotor1, LeftMotor2);

  private final WPI_TalonSRX RightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMotor2 = new WPI_TalonSRX(4);
  SpeedControllerGroup rightmotors = new SpeedControllerGroup(RightMotor1, RightMotor2);

  private final WPI_VictorSPX LiftMotor1 = new WPI_VictorSPX(6);
  private final WPI_VictorSPX LiftMotor2 = new WPI_VictorSPX(7);
  private final WPI_VictorSPX BallMotor = new WPI_VictorSPX(8);
  private final WPI_VictorSPX PanelSpinner = new WPI_VictorSPX(9);

  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;
  public int AutoCounter = 0;

  private final Joystick joy1 = new Joystick(0);

  VictorSP Agitator = new VictorSP(0);

  DifferentialDrive drivetrain = new DifferentialDrive(leftmotors, rightmotors);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Sensor Max Range (inches)", 196);

    LiftMotor1.setInverted(true);
    LiftMotor2.setInverted(true);
    LiftMotor1.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen,
        1);
    LiftMotor2.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen,
        1);
    DeploySpinner = new DoubleSolenoid(4, 5);
    c = new Compressor(0);
    c.setClosedLoopControl(true);
    try {
      ahrs = new AHRS(SPI.Port.kMXP);
      ahrs.enableLogging(true);
    } catch (RuntimeException ex) {
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
    SmartDashboard.putNumber("Sensor 1 Range (Inches)", ultrasonicSensorOneRange);
    voltageScaleFactor = 5 / RobotController.getVoltage5V();
    ultrasonicSensorOneRange = (ultrasonicSensorOne.getValue() * voltageScaleFactor * 0.0492);

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
    AutoCounter = 0;

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    if (ultrasonicSensorOneRange >= 14) {
      drivetrain.curvatureDrive(-.15, 0, false);
    } else {
      AutoCounter += 1;
      drivetrain.curvatureDrive(0, 0, false);
      if (AutoCounter < 250) {
        BallMotor.set(ControlMode.PercentOutput, 1);
        Agitator.set(1);
      }
      else
      {
        BallMotor.set(ControlMode.PercentOutput, 0);
        Agitator.set(0);
      }
    }
  }

  @Override
  public void teleopPeriodic() {
    if (joy1.getRawButton(3)) {
      DeploySpinner.set(DoubleSolenoid.Value.kForward);
    } else {
      DeploySpinner.set(DoubleSolenoid.Value.kReverse);
    }

    if (joy1.getPOV() == 90) {
      PanelSpinner.set(ControlMode.PercentOutput, -1);
    }

    if (joy1.getPOV() == 270) {
      PanelSpinner.set(ControlMode.PercentOutput, 1);
    }

    if (joy1.getPOV() == -1) {
      PanelSpinner.set(ControlMode.PercentOutput, 0);
    }

    if (joy1.getRawButton(1) && (ultrasonicSensorOneRange <= 14.0 || joy1.getPOV() == 180)) {
      Agitator.set(1);
      BallMotor.set(ControlMode.PercentOutput, 1);
    } else {
      Agitator.set(0);
      BallMotor.set(ControlMode.PercentOutput, 0);
    }

    if (Math.abs(joy1.getRawAxis(4)) > .1) {
      LiftMotor1.set(ControlMode.PercentOutput, joy1.getRawAxis(4));
      LiftMotor2.set(ControlMode.PercentOutput, joy1.getRawAxis(4));
    } else {
      LiftMotor1.set(ControlMode.PercentOutput, 0);
      LiftMotor2.set(ControlMode.PercentOutput, 0);
    }
    RightMotor2.setSensorPhase(true);
    LeftMotor2.setSensorPhase(true);
    RightMotor2.getFaults(_rightfaults);
    LeftMotor2.getFaults(_leftfaults);

    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double turn = joy1.getRawAxis(0);
    double speed = ((forward - reverse) * (forward - reverse));
    if (reverse > 0) {
      speed = speed * -1;
      turn = turn * -1;
    }

    if (joy1.getRawButton(5)) {
      drivetrain.curvatureDrive(speed, -1, true);
    }

    if (joy1.getRawButton(6)) {
      drivetrain.curvatureDrive(speed, 1, true);
    }

    if ((joy1.getRawButton(5) == false) && (joy1.getRawButton(6) == false)) {
      drivetrain.curvatureDrive(speed, turn, false);
    }

    drivetrain.setSafetyEnabled(true);

  }

  @Override
  public void testPeriodic() {
  }
}
