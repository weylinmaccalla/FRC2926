/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.wpilibj.util.Color;
import vision2926.Vision;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends TimedRobot {
  Faults _rightfaults = new Faults();
  Faults _leftfaults = new Faults();

  AHRS ahrs;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double centerX = 0.0;
  private double area = 0.0;


  private final Object imgLock = new Object();

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  PowerDistributionPanel pdp = new PowerDistributionPanel(5);

  Compressor c;
  DoubleSolenoid Piston;

  private final WPI_TalonSRX LeftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMotor2 = new WPI_TalonSRX(2);
  
  SpeedControllerGroup leftmotors = new SpeedControllerGroup(LeftMotor1, LeftMotor2);

  private final WPI_TalonSRX RightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMotor2 = new WPI_TalonSRX(4);
  SpeedControllerGroup rightmotors = new SpeedControllerGroup(RightMotor1, RightMotor2);

  private final Joystick joy1 = new Joystick(0);

  DifferentialDrive drivetrain = new DifferentialDrive(leftmotors, rightmotors);

  double last_world_linear_accel_x;
  double last_world_linear_accel_y;
  CameraServer server;

  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
   
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new Vision(), pipeline -> {
      if (!pipeline.filterContoursOutput().isEmpty()) {
        Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
        synchronized (imgLock) {
          area = r.width*r.height;
          centerX = r.x + (r.width / 2);

        }
      }
    });

    visionThread.start();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    CameraServer.getInstance().startAutomaticCapture();

    Piston = new DoubleSolenoid(0, 1);
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
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
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
    double centerX;

    synchronized (imgLock) {
      centerX = this.centerX;
    }
    double turn = centerX - (IMG_WIDTH / 2);
    drivetrain.curvatureDrive(0.25, turn * 0.003, true);

  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("area", area);

    if (joy1.getRawButton(1) && area > 1000)
    {
     
      double centerX;

      synchronized (imgLock) {
        centerX = this.centerX;
      }
      double turn = centerX - (IMG_WIDTH / 2);
      drivetrain.curvatureDrive(0.25, turn * 0.003, true);
    }
     if(joy1.getRawButton(3))
     {
      Piston.set(DoubleSolenoid.Value.kForward);
     }
     else
     {
       Piston.set(DoubleSolenoid.Value.kReverse);
     }

    RightMotor2.setSensorPhase(true);
    LeftMotor2.setSensorPhase(true);
    RightMotor2.getFaults(_rightfaults);
    LeftMotor2.getFaults(_leftfaults);
    SmartDashboard.putNumber("Right Sensor Vel:", RightMotor2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Left Sensor Vel:", LeftMotor2.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Sensor Pos:", RightMotor2.getSelectedSensorPosition());
    SmartDashboard.putNumber("LeftSensor Pos:", LeftMotor2.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Out %", RightMotor2.getMotorOutputPercent());
    SmartDashboard.putNumber("Left Out %", LeftMotor2.getMotorOutputPercent());
    SmartDashboard.putBoolean("Left Out Of Phase:", _leftfaults.SensorOutOfPhase);
    SmartDashboard.putBoolean("Right Out Of Phase:", _rightfaults.SensorOutOfPhase);
    SmartDashboard.putBoolean("NavX Connected?", ahrs.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating?", ahrs.isCalibrating());
    SmartDashboard.putNumber("Compass Heading", ahrs.getCompassHeading());
    SmartDashboard.putBoolean("Under Max Pressure", c.getPressureSwitchValue());
    SmartDashboard.putNumber("Total Power Consumption", pdp.getTotalCurrent());
    SmartDashboard.putNumber("Right Motor 1 Amps", pdp.getCurrent(1));
    SmartDashboard.putNumber("Right Motor 2 Amps", pdp.getCurrent(2));
    SmartDashboard.putNumber("Left Motor 1 Amps", pdp.getCurrent(14));
    SmartDashboard.putNumber("Left Motor 2 Amps", pdp.getCurrent(15));

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

    drivetrain.setSafetyEnabled(false);

  }
 
  @Override
  public void testPeriodic() {
  }
}
