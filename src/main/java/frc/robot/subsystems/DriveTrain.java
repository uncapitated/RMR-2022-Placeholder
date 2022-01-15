// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;;

public class DriveTrain extends SubsystemBase {
  
  private WPI_VictorSPX frontLeft;
  private WPI_VictorSPX backLeft;

  private WPI_VictorSPX frontRight;
  private WPI_VictorSPX backRight;

  private MotorControllerGroup left;
  private MotorControllerGroup right;

  private DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain()
  {
    // setup left drive
    frontLeft = new WPI_VictorSPX(Drive.FRONT_LEFT);
    backLeft = new WPI_VictorSPX(Drive.BACK_LEFT);

    // must be inverted
    backLeft.setInverted(true);

    left = new MotorControllerGroup(frontLeft, backLeft);
    left.setInverted(true);

    // setup right drive
    frontRight = new WPI_VictorSPX(Drive.FRONT_RIGHT);
    backRight = new WPI_VictorSPX(Drive.BACK_RIGHT);

    right = new MotorControllerGroup(frontRight, backRight);

    drive = new DifferentialDrive(left, right);
  }

  public void set(double speed, double rotation)
  {
    drive.arcadeDrive(speed, rotation);
  }

  /**
   * @return the current percent output of the right side of the robot
   */
  public double getRightPercent() {
    return left.get();
  }

  /**
   * @return the current percent output of the left side of the robot
   */
  public double getLeftPercent() {
    return left.get();
  }

  /**
   * sets the motors to coast mode to prevent current usage
   */
  public void setCoast()
  {
    VictorSPX[] motors = {frontLeft, frontRight, backLeft, backRight};
    for (VictorSPX motor : motors) {
      motor.setNeutralMode(NeutralMode.Coast);
    }

    set(0, 0);
  }

  /**
   * sets the motors to break mode to enable better control
   */
  public void setBreak()
  {
    VictorSPX[] motors = {frontLeft, frontRight, backLeft, backRight};
    for (VictorSPX motor : motors) {
      motor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public DifferentialDrive getDrive() {
    return drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
