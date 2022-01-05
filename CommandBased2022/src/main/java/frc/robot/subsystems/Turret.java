// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TurretState;

public class Turret extends SubsystemBase {
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

  private final VictorSP turretAngleMotor = new VictorSP(0);
  private final WPI_VictorSPX ballThrowerMotor = new WPI_VictorSPX(8);

  /** Creates a new Turret. */
  public Turret() {
    // Code below defines the encoder for the turret
    enc = new Encoder(0, 1);
    enc.setDistancePerPulse(((Math.PI * whd / cpr) / 17597.238550001526) * 360); // distance per pulse is pi* (wheel //
                                                                                 // diameter / counts per revolution)
  }

  public void TurretStateMachine() {
    // Code below allows us to grab NetworkTable values from Raspberry Pi
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("photonvision"); // .Logitech,_Inc._Webcam_C260

    // Code below defines variables and grabs NetworkTable values from Rasberry Pi
    yaw = table.getEntry("Logitech,_Inc._Webcam_C260/targetYaw");
    hasTarget = table.getEntry("Logitech,_Inc._Webcam_C260/hasTarget");
    targetArea = table.getEntry("Logitech,_Inc._Webcam_C260/targetArea");

    // Code below reports important values on drive station
    SmartDashboard.putNumber("Encoder Angle", enc.getDistance());
    SmartDashboard.putNumber("Yaw", (double) yaw.getNumber(0));
    SmartDashboard.putNumber("Target Area", (double) targetArea.getNumber(0));
    SmartDashboard.putBoolean("Has Target?", hasTarget.getBoolean(false));
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

  public void launchCells() {
      ballThrowerMotor.set(1);
  }
  public void stopLauncher(){
      ballThrowerMotor.set(0);
  }

  public void resetStateMachine() {
    turretState = TurretState.HomingLeft;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
