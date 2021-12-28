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
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  int LoopCounter = 0;

  public void TurretStateMachine() {
    if (turretState == TurretState.HomingLeft) {
      LoopCounter++;
      SmartDashboard.putNumber("LoopCounter", LoopCounter);
      TurretSpinner.set(-.2);
      if (!leftlimitSwitch.get()) {
        enc.reset();
        turretState = TurretState.HomingRight;
      }
    } else if (turretState == TurretState.HomingRight) {
      TurretSpinner.set(.2);
      if (!rightlimitSwitch.get()) {
        MaxTurretAngle = enc.getDistance();
        SmartDashboard.putNumber("Encoder Max Angle", enc.getDistance());
        CenterTurretAngle = MaxTurretAngle / 2;
        turretState = TurretState.SearchLeft;
      }
    } else if (turretState == TurretState.SearchLeft) {
      TurretSpinner.set(-.5);

      if (enc.getDistance() <= 10) {
        turretState = TurretState.SearchRight;
      } else if (HasTarget.getBoolean(false) == true) {
        turretState = TurretState.Track;
      }
    } else if (turretState == TurretState.SearchRight) {
      TurretSpinner.set(.5);

      if (enc.getDistance() >= MaxTurretAngle - 10) {
        turretState = TurretState.SearchLeft;
      } else if (HasTarget.getBoolean(false) == true) {
        turretState = TurretState.Track;
      }
    } else if (turretState == TurretState.Track) {
    
      double setpoint = enc.getDistance() + (double) Yaw.getNumber(0);

      if (setpoint <= 5)
      {
        setpoint = 5;
      }
      else if (setpoint >= MaxTurretAngle-2)
      {
        setpoint = MaxTurretAngle-2;
      }    
      Double MotorSpeed = pid.calculate(enc.getDistance(), setpoint);
      TurretSpinner.set(MotorSpeed);
      
      if (HasTarget.getBoolean(false) == false) {
        turretState = TurretState.SearchLeft;
      }
    }
  }

  NetworkTableEntry Yaw;
  NetworkTableEntry HasTarget;
  NetworkTableEntry TargetArea;
  TurretState turretState = TurretState.HomingLeft;
  double MaxTurretAngle = 10000;
  double CenterTurretAngle = 10000;
  DigitalInput leftlimitSwitch = new DigitalInput(2);
  DigitalInput rightlimitSwitch = new DigitalInput(3);

  Encoder enc;
  PIDController pid = new PIDController(.1, 0, 0);
  private static final double cpr = 7 / 4;
  private static final double whd = 1.5;

  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  AHRS ahrs;

  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  Compressor c;
  DoubleSolenoid DeploySpinner;
  DoubleSolenoid Bridge;

  private final WPI_TalonSRX LeftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMotor2 = new WPI_TalonSRX(2);

  SpeedControllerGroup leftmotors = new SpeedControllerGroup(LeftMotor1, LeftMotor2);

  private final WPI_TalonSRX RightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMotor2 = new WPI_TalonSRX(4);
  SpeedControllerGroup rightmotors = new SpeedControllerGroup(RightMotor1, RightMotor2);

  private final WPI_VictorSPX BallThrowerMotor = new WPI_VictorSPX(8);

  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;
  public int AutoCounter = 0;

  private final Joystick joy1 = new Joystick(0);

  VictorSP TurretSpinner = new VictorSP(0);

  DifferentialDrive drivetrain = new DifferentialDrive(leftmotors, rightmotors);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    enc = new Encoder(0, 1);
    enc.setDistancePerPulse(((Math.PI * whd / cpr) / 17597.238550001526) * 360); // distance per pulse is pi* (wheel
                                                                                 // diameter / counts per revolution)
    SmartDashboard.putNumber("Sensor Max Range (inches)", 196);
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
    SmartDashboard.putNumber("Encoder Angle", enc.getDistance());
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("photonvision"); // .Logitech,_Inc._Webcam_C260
    Yaw = table.getEntry("Logitech,_Inc._Webcam_C260/targetYaw");
    HasTarget = table.getEntry("Logitech,_Inc._Webcam_C260/hasTarget");
    TargetArea = table.getEntry("Logitech,_Inc._Webcam_C260/targetArea");
    SmartDashboard.putNumber("Yaw", (double) Yaw.getNumber(0));
    SmartDashboard.putBoolean("Has Target?", HasTarget.getBoolean(false));
    SmartDashboard.putNumber("Target Area", (double) TargetArea.getNumber(0));

    if (joy1.getRawButton(3)) {
      BallThrowerMotor.set(1);
    } else {
      BallThrowerMotor.set(0);
    }

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

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    TurretStateMachine();
    RightMotor2.setSensorPhase(true);
    LeftMotor2.setSensorPhase(true);
    RightMotor2.getFaults(_rightfaults);
    LeftMotor2.getFaults(_leftfaults);

    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double turn = joy1.getRawAxis(0);
    double speed = (forward - reverse) * (forward - reverse) * (forward - reverse);
    if (reverse > 0) {
      // speed = speed * -1;
      turn = turn * -1;
    }

    if (joy1.getRawButton(5)) {
      drivetrain.curvatureDrive(speed, -.4, true);
    }

    if (joy1.getRawButton(6)) {
      drivetrain.curvatureDrive(speed, .4, true);
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