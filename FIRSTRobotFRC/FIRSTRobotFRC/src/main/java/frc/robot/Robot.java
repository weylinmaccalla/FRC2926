/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This is a demo program providing a real-time display of navX MXP values.
 *
 * In the operatorControl() method, all data from the navX sensor is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles - Compass Heading and 9-Axis Fused Heading
 * (requires Magnetometer calibration) - Linear Acceleration Data - Motion
 * Indicators - Estimated Velocity and Displacement - Quaternion Data - Raw
 * Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for
 * debugging connectivity issues after initial installation of the navX MXP
 * sensor.
 *
 */
public class Robot extends TimedRobot {
  PowerDistributionPanel pdp = new PowerDistributionPanel(5);
  Compressor c;
  DoubleSolenoid Piston;
  AHRS ahrs;
  private final WPI_TalonSRX LeftMotor1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX LeftMotor2 = new WPI_TalonSRX(2);
  
  SpeedControllerGroup leftmotors = new SpeedControllerGroup(LeftMotor1, LeftMotor2);

  private final WPI_TalonSRX RightMotor1 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX RightMotor2 = new WPI_TalonSRX(4);
  SpeedControllerGroup rightmotors = new SpeedControllerGroup(RightMotor1, RightMotor2);

  private final Joystick joy1 = new Joystick(0);

  DifferentialDrive drivetrain = new DifferentialDrive(leftmotors, rightmotors);

  boolean quickturn = false;
   
  double leftSlow = 0.24;
  double rightSlow = -0.24;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25;

  final static double kCollisionThreshold_DeltaG = 0.5f;
  double last_world_linear_accel_x;
  double last_world_linear_accel_y;
  CameraServer server;
  

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
    
    Piston = new DoubleSolenoid(0,1);
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

    if (Math.abs(ahrs.getCompassHeading()) <= 3) {
      leftmotors.set(leftSlow - (ahrs.getCompassHeading()) / 15);
      rightmotors.set(rightSlow - (ahrs.getCompassHeading()) / 15);
    } else if (Math.abs(ahrs.getCompassHeading()) < 10) {
      if (ahrs.getCompassHeading() > 0) {
        leftmotors.set(leftSlow);
        rightmotors.set(1.1 * rightSlow);
      } else if (ahrs.getCompassHeading() < 0) {
        leftmotors.set(1.1 * leftSlow);
        rightmotors.set(rightSlow);
      }
    } else if (ahrs.getCompassHeading() > 0) {
      while (ahrs.getCompassHeading() > 10 && isAutonomous()) {
        leftmotors.set(-rotateSpeed);
        rightmotors.set(-rotateSpeed);
      }
      while (ahrs.getCompassHeading() > 0 && isAutonomous()) {
        leftmotors.set(-rotateSpeedSlow);
        rightmotors.set(-rotateSpeedSlow);
      }
      while (ahrs.getCompassHeading() < 0 && isAutonomous()) {
        leftmotors.set(rotateSpeedSlow);
        rightmotors.set(rotateSpeedSlow);
      }
    } else {
      while (ahrs.getCompassHeading() < -10 && isAutonomous()) {
        leftmotors.set(rotateSpeed);
        rightmotors.set(rotateSpeed);
      }
      while (ahrs.getCompassHeading() < 0 && isAutonomous()) {
        leftmotors.set(rotateSpeedSlow);
        rightmotors.set(rotateSpeedSlow);
      }
      while (ahrs.getCompassHeading() > 0 && isAutonomous()) {
        leftmotors.set(-rotateSpeedSlow);
        rightmotors.set(-rotateSpeedSlow);
      }
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
   
    /* Display 6-axis Processed Angle Data */
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
    double speed = forward - reverse;

    if (joy1.getRawButton(2) == true)
    {
      Piston.set(DoubleSolenoid.Value.kForward);
    }
    else
    {
      Piston.set(DoubleSolenoid.Value.kReverse);
    }
    if (joy1.getRawButton(5) == true)
    {
      drivetrain.curvatureDrive(.25, -1, true);
    }
    else;
    {
      drivetrain.curvatureDrive(speed, turn, false);
    }
    if (joy1.getRawButton(6) == true)
    {
      drivetrain.curvatureDrive(.5, 1, true);
    }
    else
    {
      drivetrain.curvatureDrive(speed, turn, false);
    }
    //drivetrain.curvatureDrive(speed, turn, false);
    drivetrain.setSafetyEnabled(false);

    
           boolean collisionDetected = false;
          
          double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
          double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
          last_world_linear_accel_x = curr_world_linear_accel_x;
          SmartDashboard.putNumber("X Axis Jerk", currentJerkX);
          double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
          double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
          last_world_linear_accel_y = curr_world_linear_accel_y;
          SmartDashboard.putNumber("Y Axis Jerk", currentJerkY);
          
          if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
               ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
              collisionDetected = true;
          }
          SmartDashboard.putBoolean(  "CollisionDetected", collisionDetected);

        
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
