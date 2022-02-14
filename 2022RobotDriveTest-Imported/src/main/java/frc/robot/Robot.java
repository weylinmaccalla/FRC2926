// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
 PIDController shooterPID = new PIDController(.00005, .00005, 0);
  // CAN ID 0 = Power Board, CAN ID 1 = Shooter, CAN ID 2 = Feeder
  private final Joystick joy1 = new Joystick(0);

  DoubleSolenoid intake;
  
  private final CANSparkMax ballCollector = new CANSparkMax(6, MotorType.kBrushless);

  private final CANSparkMax leftMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(3, MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

  private final CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(1, MotorType.kBrushless);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

  DifferentialDrive drivetrain;

  CANSparkMax Shooter = new CANSparkMax(8, MotorType.kBrushless);
  CANSparkMax Feeder = new CANSparkMax(7, MotorType.kBrushed);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  int shooterTimer = 0;
  String ball = "";

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Feeder.setInverted(true);
    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    intake = new DoubleSolenoid(5, PneumaticsModuleType.CTREPCM, 1,0);
    Shooter.setInverted(true);
    Feeder.setInverted(true);
    ballCollector.setInverted(true);
    drivetrain = new DifferentialDrive(leftMotors, rightMotors);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    int proximity = m_colorSensor.getProximity();
    int redColor = m_colorSensor.getRed();
    int blueColor = m_colorSensor.getBlue();
    SmartDashboard.putString("Ball", ball);

    boolean quickTurn = false;
    double CorrectedShooterSpeed = shooterPID.calculate(Shooter.getEncoder().getVelocity(), 1300);
    SmartDashboard.putNumber("PID Shooter Speed", CorrectedShooterSpeed);

    double reverse = joy1.getRawAxis(2);
    double forward = joy1.getRawAxis(3);
    double speed = forward - reverse;
    double turn = -joy1.getRawAxis(0);
    double adjustedSpeed = Math.pow(speed, 3);

    double shooterSpeed = Shooter.getEncoder().getVelocity();
    SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
    SmartDashboard.putNumber("Shooter Timer", shooterTimer);
    if (proximity > 250) {
      if (redColor > blueColor) {
        ball = "Red Ball";
      } else {
        ball = "Blue Ball";
      }
    } else {
      ball = "No Ball";
    }
   
    if (joy1.getRawButton(2)) {
      if (proximity < 250) {
        shooterTimer++;
        Feeder.set(.4);

        if (shooterTimer > 50) {
          Shooter.set(0);
        }
      } else if (proximity > 250) {
        if (shooterSpeed < 1300) { //1300 RPM works well for low
          Shooter.set(.3);
          Feeder.set(0);
        } else {
          shooterTimer = 0;
          Shooter.set(.3);
          Feeder.set(1);
        }
      }
    } else {
      Shooter.set(0);
      Feeder.set(0);
    }

    if (joy1.getRawButton(1)) 
    {

      intake.set(DoubleSolenoid.Value.kForward);

      if (proximity < 250)
      {
      ballCollector.set(.65);
      Feeder.set(.4);
      }
      else
      {
      ballCollector.set(.65);
      Feeder.set(0);
      }
    } 
    else
    {
      intake.set(DoubleSolenoid.Value.kReverse);

      ballCollector.set(0);
    }


    if (joy1.getRawButton(5)) {
      quickTurn = true;
    } else {
      quickTurn = false;
    }

    if (joy1.getRawButton(3))
    {
      Shooter.set(CorrectedShooterSpeed);

    }

    drivetrain.curvatureDrive(adjustedSpeed, turn, quickTurn);
    drivetrain.setSafetyEnabled(true);


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
