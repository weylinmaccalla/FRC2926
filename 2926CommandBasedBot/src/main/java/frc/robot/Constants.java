/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    XboxController xboxController= new XboxController(1);
    double reverse = xboxController.getRawAxis(2);
    double forward = xboxController.getRawAxis(3);
    double turn = xboxController.getRawAxis(0);
    double speed = forward - reverse;
}
