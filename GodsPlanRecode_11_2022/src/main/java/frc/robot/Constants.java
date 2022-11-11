// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class DriveSubsystemConstants{
        public static final int rightMotor1_CanID = 1;
        public static final int rightMotor2_CanID = 2;
        public static final int leftMotor1_CanID = 3;
        public static final int leftMotor2_CanID = 4;
    }

    public final class ClimberSubsystemConstants{
        public static final int LeftClimberMotor_CanID = 10;
        public static final int RightClimberMotor_CanID = 9;
        public static final int RightLimitSwitch_DIO = 1;
        public static final int LeftLimitSwitch_DIO = 0;
    }

    public final class IntakeSubsystemConstants{
        public static final int intake_CanID = 6;
        public static final int Right_Solenoid = 0;
        public static final int Lift_Solenoid = 1;
    }

    public final class ShooterSubsystemConstants{
        public static final int lancher_canID=7;
         public static final int brush_canID=8;
    }
}
