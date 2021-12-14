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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Robot extends TimedRobot {
  NetworkTableEntry Yaw;

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
    enc.setDistancePerPulse(((Math.PI * whd / cpr) / 17597.238550001526) * 360); // distance per pulse is pi* (wheel diameter / counts per revolution)
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
    NetworkTable table = inst.getTable("photonvision"); //.Logitech,_Inc._Webcam_C260
    Yaw = table.getEntry("Logitech,_Inc._Webcam_C260/targetYaw");
    SmartDashboard.putNumber("Yaw", (double) Yaw.getNumber(0));

    if (joy1.getRawButton(3))
    {
      BallThrowerMotor.set(1);
    }
      else
    {
      BallThrowerMotor.set(0);
    }
    double OppositeAmp = joy1.getRawAxis(4); // Left and Right on Joystick
    double AdjacentAmp = joy1.getRawAxis(5); // Up and Down on Joystick
    double AngleRadians = Math.atan(Math.abs(OppositeAmp) / Math.abs(AdjacentAmp)); // Obtaining angle using Inverse
                                                                                    // Tan, using the joysticks to
                                                                                    // obtain side lengths
    /*double AngleDegrees = Math.toDegrees(AngleRadians); // Converting Radians to Degrees
    if (OppositeAmp < 0 && AdjacentAmp < 0) // Gives you angle in second quadrant
    {
      AngleDegrees = 360 - AngleDegrees;
    } else if (OppositeAmp < 0 && AdjacentAmp > 0) // Gives you angle in third quadrant
    {
      AngleDegrees += 180;
    } else if (OppositeAmp > 0 && AdjacentAmp > 0) // Gives you angle in fourth quadrant
    {
      AngleDegrees = 180 - AngleDegrees;
    }
    if (OppositeAmp == 0 && AdjacentAmp > 0) // Special case if joystick is down
    {
      AngleDegrees = 180;
    }
    if (OppositeAmp < 0 && AdjacentAmp == 0) // Special case if joystick is pointed to left
    {
      AngleDegrees = 270;
    }

    SmartDashboard.putNumber("Degrees", AngleDegrees);
    double MotorSpeed = pid.calculate(enc.getDistance(), AngleDegrees);
    */
  
    //if (MotorSpeed != 0)
  //  {
      //TurretSpinner.set(MotorSpeed);

    //}
      if (joy1.getRawButton(2)) {
        
      Double MotorSpeed = pid.calculate(enc.getDistance(), (enc.getDistance()+ (double) Yaw.getNumber(0)));
      SmartDashboard.putNumber("Motor Speed", MotorSpeed);
      SmartDashboard.putNumber("Delta Angle", (enc.getDistance()+ (double) Yaw.getNumber(0)));
      SmartDashboard.putNumber("Motor Speed2", MotorSpeed);
      TurretSpinner.set(MotorSpeed);

    } 
    else
    {
      TurretSpinner.set(0);
    }


    if (joy1.getPOV() == 90) {
      TurretSpinner.set(0.5);
    }
    else if (joy1.getPOV() == 270)
    {
      TurretSpinner.set(-0.5);
    }
    if (joy1.getRawButton(1)) {
      enc.reset();

    SmartDashboard.putNumber("Sensor 1 Range (Inches)", ultrasonicSensorOneRange);
    voltageScaleFactor = 5 / RobotController.getVoltage5V();
    ultrasonicSensorOneRange = (ultrasonicSensorOne.getValue() * voltageScaleFactor * 0.0492);

  }
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
    RightMotor2.setSensorPhase(true);
    LeftMotor2.setSensorPhase(true);
    RightMotor2.getFaults(_rightfaults);
    LeftMotor2.getFaults(_leftfaults);

    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double turn = joy1.getRawAxis(0);
    double speed = Math.sqrt(((forward - reverse) * (forward - reverse)));
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
