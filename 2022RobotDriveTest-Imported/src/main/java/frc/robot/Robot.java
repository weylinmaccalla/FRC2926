// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxPIDController;

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

  public AnalogInput ultrasonicSensorOne = new AnalogInput(0);
  public double ultrasonicSensorOneRange = 0;
  public double voltageScaleFactor = 1;


  private SparkMaxPIDController shooterVelocityPID;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, shooterSetpoint;

  private final XboxController operatorController = new XboxController(0);
  private final XboxController driverController = new XboxController(1);


  DoubleSolenoid intake;

  private final CANSparkMax ballCollector = new CANSparkMax(6, MotorType.kBrushless);

  private final CANSparkMax leftMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(3, MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

  private final CANSparkMax rightMotor1 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(1, MotorType.kBrushless);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

  DifferentialDrive drivetrain;

  private final CANSparkMax feeder = new CANSparkMax(7, MotorType.kBrushed);
  private final CANSparkMax shooter = new CANSparkMax(8, MotorType.kBrushless);

  private final CANSparkMax rightClimber = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax leftClimber = new CANSparkMax(10, MotorType.kBrushless);

  DigitalInput rightClimberLimitSwitch = new DigitalInput(1);
  DigitalInput leftClimberLimitSwitch = new DigitalInput(0);


  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  int shooterTimer = 0;
  int intakeTimer = 150;
  int reverseTimer = 0;
  String ball = "";
  double intakeSpeed = .3;
  boolean isRightSwitchBumped = false;
  boolean isLeftSwitchBumped = false;

  SendableChooser<Integer> autonomousChooser = new SendableChooser<>();

int proximity;
double shooterSpeed;
double voltage_scale_factor;
double currentDistanceCentimeters;
double currentDistanceInches;


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    autonomousChooser.setDefaultOption("Drive, Shoot, Reverse", 1);
    autonomousChooser.addOption("Drive, Shoot, Stop", 2);
    autonomousChooser.addOption("Reverse", 3);
    autonomousChooser.addOption("Nothing",4);

    SmartDashboard.putData("Option",autonomousChooser);

    shooterVelocityPID = shooter.getPIDController();

    // PID coefficients

    kP = .0002;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.0002;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;
    shooterSetpoint = 1300;

    // set PID coefficients
    shooterVelocityPID.setP(kP, 0);
    shooterVelocityPID.setI(kI, 0);
    shooterVelocityPID.setD(kD, 0);
    shooterVelocityPID.setIZone(kIz, 0);
    shooterVelocityPID.setFF(kFF, 0);
    shooterVelocityPID.setOutputRange(kMinOutput, kMaxOutput, 0);

    feeder.setInverted(true);
    shooter.setInverted(true);
    ballCollector.setInverted(true);

   

    intake = new DoubleSolenoid(5, PneumaticsModuleType.CTREPCM , 1 , 0);    
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
    proximity = colorSensor.getProximity();
    double rawValue = ultrasonicSensorOne.getValue();

    shooterSpeed = shooter.getEncoder().getVelocity();

    
    //voltage_scale_factor allows us to compensate for differences in supply voltage.

double voltage_scale_factor = 5/RobotController.getVoltage5V();
 currentDistanceCentimeters = rawValue * voltage_scale_factor * 0.125;
 currentDistanceInches = rawValue * voltage_scale_factor * 0.0492;

 SmartDashboard.putNumber("Ultrasonic Sensor Distance (In.)", currentDistanceInches);  
 SmartDashboard.putNumber("Ultrasonic Sensor Distance (Cm.)", currentDistanceCentimeters);  

    SmartDashboard.putNumber("Right Climber Encoder", rightClimber.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Climber Encoder", leftClimber.getEncoder().getPosition());

    SmartDashboard.putNumber("Autonomous Program", autonomousChooser.getSelected());
    int proximity = colorSensor.getProximity();
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
  isRightSwitchBumped = false;
  isLeftSwitchBumped = false;
  reverseTimer = 0;


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (rightClimberLimitSwitch.get() && isRightSwitchBumped == false)
    {
      rightClimber.set(.25);
    }
    else if (isRightSwitchBumped == false)
    {
      rightClimber.set(0);
      rightClimber.getEncoder().setPosition(0);
      isRightSwitchBumped = true;
    }

    if (leftClimberLimitSwitch.get() && isLeftSwitchBumped == false)
    {
      leftClimber.set(-.25);
    }
    else if (isLeftSwitchBumped == false)
    {
      leftClimber.set(0);
      leftClimber.getEncoder().setPosition(0);
      isLeftSwitchBumped = true;
    }

      if(isLeftSwitchBumped == true)
      {
        if(leftClimber.getEncoder().getPosition() < 15)
      {
        leftClimber.set(.25);
      }
      else
      {
        leftClimber.set(0);
      }
      }

      if(isRightSwitchBumped == true){

        if(rightClimber.getEncoder().getPosition() > -15)
        {
          rightClimber.set(-.25);
        }
        else
        {
          rightClimber.set(0);
        }
      }


    if (autonomousChooser.getSelected() == 1) // Drive, Shoot, Reverse
    {

      if (currentDistanceInches > 15.0 && proximity > 250)
      {
        drivetrain.curvatureDrive(.1, 0, false);
      }
      else if (proximity > 250)
      {
      drivetrain.curvatureDrive(0, 0, false);
      shooterVelocityPID.setReference(1300.0, CANSparkMax.ControlType.kVelocity, 0);
      double error = 1300 - shooterSpeed;

      if (Math.abs(error) < 70) {
        feeder.set(1);
        ballCollector.set(intakeSpeed);
      }
      }

      if (proximity < 250)
      {
        reverseTimer++;
        shooter.set(0);
        feeder.set(0);
        ballCollector.set(0);

        if (reverseTimer > 200)
        {
          drivetrain.curvatureDrive(0, 0, false);
        }
        else
        {
          drivetrain.curvatureDrive(-.2, 0, false);
        }
      }
    }
    else if (autonomousChooser.getSelected() == 2) //Drive, Shoot, Stop
    {
      
      if (currentDistanceInches > 15.0 && proximity > 250)
      {
        drivetrain.curvatureDrive(.1, 0, false);
      }
      else if (proximity > 250)
      {
      drivetrain.curvatureDrive(0, 0, false);
      shooterVelocityPID.setReference(1300.0, CANSparkMax.ControlType.kVelocity, 0);
      double error = 1300 - shooterSpeed;

      if (Math.abs(error) < 70) {
        feeder.set(1);
        ballCollector.set(intakeSpeed);
      }
      }

      if (proximity < 250)
      {
        shooter.set(0);
        feeder.set(0);
        ballCollector.set(0);
        drivetrain.curvatureDrive(0, 0, false);
      }
    }
    else if (autonomousChooser.getSelected() == 3) // Reverse
    {
      reverseTimer++;
      if (reverseTimer > 200)
      {
        drivetrain.curvatureDrive(0, 0, false);
      }
      else
      {
        drivetrain.curvatureDrive(-.2, 0, false);
      }
    }
    else if (autonomousChooser.getSelected() == 4) // Do Nothing
    {
     drivetrain.curvatureDrive(0, 0, false);
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operatorController control. */
  @Override
  public void teleopPeriodic() {

    int redColor = colorSensor.getRed();
    int blueColor = colorSensor.getBlue();
    SmartDashboard.putString("Ball", ball);

    boolean quickTurn = false;

    double reverse = driverController.getRawAxis(2);
    double forward = driverController.getRawAxis(3);
    double speed = Math.pow((forward - reverse),7);
    double turn = -driverController.getRawAxis(0);

    

    //Inverts turning when reversing the robot
    if (speed < 0)
     {
      turn = -1 * turn;
    }

    // Lets make backwards speed bring the climber down, forward speed raise the climber
    if (operatorController.getPOV() == 0)
    {
      if (rightClimber.getEncoder().getPosition() > -570)
      {
      rightClimber.set(-1);
      }
      else
      {
      rightClimber.set(0);
      }
      if (leftClimber.getEncoder().getPosition() < 570){
      leftClimber.set(1);  
      }
        else
      {
      leftClimber.set(0);
      }

    }
    else if (operatorController.getPOV() == 180)
    {
      if (rightClimber.getEncoder().getPosition() < -15){
      rightClimber.set(1);
      }
       else
      {
      rightClimber.set(0);
      }
      if (leftClimber.getEncoder().getPosition() > 15){
      leftClimber.set(-1);
      }
      else
      {
      leftClimber.set(0);
      }
    }
    else
    {
      rightClimber.set(0);
      leftClimber.set(0);
    }

    if (driverController.getLeftBumper()) 
    {
      quickTurn = true;
      turn = .4;
    } 
    
    if (driverController.getRightBumper())
    {
      quickTurn = true;
      turn = -.4;
    }
    

    drivetrain.curvatureDrive(speed, turn, quickTurn);


    if (proximity > 250) {
      if (redColor > blueColor) 
      {
        ball = "Red Ball";
      } else {
        ball = "Blue Ball";
      }
      } 
    else 
      {
      ball = "No Ball";
      }

      
    if (operatorController.getLeftBumper())
    {
      feeder.set(-.4);
    }
    else
    {
      feeder.set(0);
    }

    if (operatorController.getXButton()){
      shooter.set(1);
    }
    else
    {
      shooter.set(0);
    }

    if (operatorController.getAButton()) 
    {
      intakeTimer = 0;
      intake.set(DoubleSolenoid.Value.kForward);
      ballCollector.set(intakeSpeed);
    
    if (proximity < 250) 
    {
        feeder.set(.4);
    }
      else 
    {
      if (!operatorController.getLeftBumper())
    {
        feeder.set(0);
    }
    }
    } 
      else 
    {
      intakeTimer++;
      intake.set(DoubleSolenoid.Value.kReverse);
      if (intakeTimer > 150)
      {
        ballCollector.set(0);
        if (!operatorController.getLeftBumper()){
          feeder.set(0);
          }
      }
      else
      {
        ballCollector.set(intakeSpeed);
        if (proximity < 250) {
          feeder.set(.4);
        } else {
          if (!operatorController.getLeftBumper()){
            feeder.set(0);
            }
         }      
      }
    }



    // Y Button on operatorController Controller
    /*
    if (operatorController.getRawButton(4)) {
      ballCollector.set(intakeSpeed);

      if (proximity < 250) {
        feeder.set(.4);
      } else {
        feeder.set(0);
      }
    } else {
      ballCollector.set(0);
      feeder.set(0);
    }
*/

    if (operatorController.getBButton()) {

      if (proximity < 250) {
        shooterTimer++;
        feeder.set(.4);
        ballCollector.set(intakeSpeed);

        if (shooterTimer > 50) {

          shooter.set(0);

        }
      } 
      else if (proximity > 250) 
      {

        SmartDashboard.putNumber("setReference Output ",
        shooterVelocityPID.setReference(1300.0, CANSparkMax.ControlType.kVelocity, 0).value);
        double error = 1300 - shooterSpeed;

        if (Math.abs(error) < 70) {
          feeder.set(1);
          ballCollector.set(intakeSpeed);
          shooterTimer = 0;
        }
      }
    } else {
      if (!operatorController.getXButton())
      {
      shooter.set(0);
      }

      
      //if (!operatorController.getRawButton(4)) {
       // feeder.set(0);
     // }
    }

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
