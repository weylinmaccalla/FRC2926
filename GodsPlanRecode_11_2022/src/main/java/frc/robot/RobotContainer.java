// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shootersubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climbersubsystem = new ClimberSubsystem();

  XboxController controller = new XboxController(0);

  JoystickButton intakebotten=new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  
  JoystickButton shoot2 =new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  
  JoystickButton lowerclimber = new JoystickButton(controller, XboxController.Button.kA.value);
  JoystickButton raiseclimber = new JoystickButton(controller, XboxController.Button.kY.value);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //driveSubsystem.setDefaultCommand(
    //  new RunCommand(
    //    () -> {
    //      driveSubsystem.arcadeDrive(0,0);
    //    }
    //    , driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // TODO: Configure button pushes for robot controller. 
    intakebotten.whenPressed(new InstantCommand(intakeSubsystem::LowerIntake, intakeSubsystem));
intakebotten.whenReleased(new InstantCommand(intakeSubsystem::RaiseIntake, intakeSubsystem));

    lowerclimber.whenPressed(new InstantCommand(climbersubsystem::LowerClimber, climbersubsystem));
    raiseclimber.whenPressed(new InstantCommand(climbersubsystem::RaiseClimber, climbersubsystem));
    
    shoot2.whenPressed(new InstantCommand(shootersubsystem::shoot, shootersubsystem));
    shoot2.whenReleased(new InstantCommand(shootersubsystem::stopshoot, shootersubsystem));
    // test.whenReleased(getAutonomousCommand())
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
