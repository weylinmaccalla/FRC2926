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
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {

  private final Joystick joy1 = new Joystick(0);

  NetworkTableEntry yaw;
  NetworkTableEntry hasTarget;
  NetworkTableEntry targetArea;

  TurretState turretState = TurretState.HomingLeft;
  double maxTurretAngle = 10000;
  DigitalInput leftLimitSwitch = new DigitalInput(2);
  DigitalInput rightLimitSwitch = new DigitalInput(3);
  Encoder enc;
  PIDController pid = new PIDController(.1, 0, 0);

  private static final double cpr = 7 / 4;
  private static final double whd = 1.5;

  VictorSP turretAngleMotor = new VictorSP(0);
  private final WPI_VictorSPX ballThrowerMotor = new WPI_VictorSPX(8);

  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(2);

  SpeedControllerGroup leftmotors = new SpeedControllerGroup(leftMotor1, leftMotor2);

  private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(4);
  SpeedControllerGroup rightmotors = new SpeedControllerGroup(rightMotor1, rightMotor2);

  DifferentialDrive drivetrain = new DifferentialDrive(leftmotors, rightmotors);

  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;

  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  public void TurretStateMachine() {
    // Code below moves turret to the sides until limit switches are bumped
    if (turretState == TurretState.HomingLeft) {
      turretAngleMotor.set(-.2);

      if (!leftLimitSwitch.get()) {
        enc.reset();
        turretState = TurretState.HomingRight;

      }
    } else if (turretState == TurretState.HomingRight) {

      turretAngleMotor.set(.2);
      if (!rightLimitSwitch.get()) {
        maxTurretAngle = enc.getDistance();
        SmartDashboard.putNumber("Encoder Max Angle", enc.getDistance());
        turretState = TurretState.SearchLeft;
      }

    }
    // Code below moves the turret to the sides until a target is found
    else if (turretState == TurretState.SearchLeft) {
      turretAngleMotor.set(-.5);

      if (enc.getDistance() <= 10) {
        turretState = TurretState.SearchRight;
      } else if (hasTarget.getBoolean(false) == true) {
        turretState = TurretState.Track;
      }
    } else if (turretState == TurretState.SearchRight) {
      turretAngleMotor.set(.5);

      if (enc.getDistance() >= maxTurretAngle - 10) {
        turretState = TurretState.SearchLeft;
      } else if (hasTarget.getBoolean(false) == true) {
        turretState = TurretState.Track;
      }

    }
    // Code below tracks a vision target, and returns to seeking one if target is
    // lost
    else if (turretState == TurretState.Track) {

      double turretSetPoint = enc.getDistance() + (double) yaw.getNumber(0);

      if (turretSetPoint <= 5) {
        turretSetPoint = 5;
      } else if (turretSetPoint >= maxTurretAngle - 2) {
        turretSetPoint = maxTurretAngle - 2;
      }
      Double motorSpeed = pid.calculate(enc.getDistance(), turretSetPoint);
      turretAngleMotor.set(motorSpeed);

      if (hasTarget.getBoolean(false) == false) {
        turretState = TurretState.SearchLeft;
      }
    }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Code below defines the encoder for the turret
    enc = new Encoder(0, 1);
    enc.setDistancePerPulse(((Math.PI * whd / cpr) / 17597.238550001526) * 360); // distance per pulse is pi* (wheel //
                                                                                 // diameter / counts per revolution)

    // Code below just tells the max distance of ultrasonic sensor
    SmartDashboard.putNumber("Sensor Max Range (inches)", 196);

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
    // Code below allows us to grab NetworkTable values from Raspberry Pi
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("photonvision"); // .Logitech,_Inc._Webcam_C260

    // Code below defines variables and grabs NetworkTable values from Rasberry Pi
    yaw = table.getEntry("Logitech,_Inc._Webcam_C260/targetYaw");
    hasTarget = table.getEntry("Logitech,_Inc._Webcam_C260/hasTarget");
    targetArea = table.getEntry("Logitech,_Inc._Webcam_C260/targetArea");
    voltageScaleFactor = 5 / RobotController.getVoltage5V();
    ultrasonicSensorOneRange = (ultrasonicSensorOne.getValue() * voltageScaleFactor * 0.0492);

    // Code below reports important values on drive station
    SmartDashboard.putNumber("Encoder Angle", enc.getDistance());
    SmartDashboard.putNumber("Yaw", (double) yaw.getNumber(0));
    SmartDashboard.putNumber("Target Area", (double) targetArea.getNumber(0));
    SmartDashboard.putNumber("Sensor 1 Range (Inches)", ultrasonicSensorOneRange);
    SmartDashboard.putBoolean("Has Target?", hasTarget.getBoolean(false));
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
    TurretStateMachine();
  }

  @Override
  public void teleopPeriodic() {
    // Code below moves turret
    TurretStateMachine();

    // Code below drives motors on wheelbase

    rightMotor2.getFaults(_rightfaults);
    leftMotor2.getFaults(_leftfaults);

    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double speed = forward - reverse;
    double turn = joy1.getRawAxis(0);
    double adjustedSpeed = Math.pow(speed, 3);

    if (reverse > 0) {
      turn = turn * -1;
    }

    if (joy1.getRawButton(5)) {
      drivetrain.curvatureDrive(adjustedSpeed, -.4, true);
    }

    if (joy1.getRawButton(6)) {
      drivetrain.curvatureDrive(adjustedSpeed, .4, true);
    }

    if ((joy1.getRawButton(5) == false) && (joy1.getRawButton(6) == false)) {
      drivetrain.curvatureDrive(adjustedSpeed, turn, false);
    }

    drivetrain.setSafetyEnabled(true);

    // Code below throws the power cells from turret
    if (joy1.getRawButton(3)) {
      ballThrowerMotor.set(1);
    } else {
      ballThrowerMotor.set(0);
    }
  }

  @Override
  public void testPeriodic() {
  }
}