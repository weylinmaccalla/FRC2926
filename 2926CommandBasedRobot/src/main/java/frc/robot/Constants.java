/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // The following are variables present within the Robot's Pneumatics System
    public static final int PCM = 0;

    public static final int SpooferForwardSol = 0;
    public static final int SpooferReverseSol = 1;
    
	public static final int EntranceReverseSol = 0;
	public static final int EntranceForwardSol = 0;
	public static final int ExitReverseSol = 0;
	public static final int ExitForwardSol = 0;

    // The following are the CAN (Controller Area Network) ID's for the Robot Motors
    
    public static final int LeftMotor1 = 1;
    public static final int LeftMotor2 = 2;

    public static final int RightMotor1 = 3;
    public static final int RightMotor2 = 4;

    public static final int LiftMotor = 5;

    public static final int HopperMotor = 6;

    public static final int SpooferMotor = 7;

}
