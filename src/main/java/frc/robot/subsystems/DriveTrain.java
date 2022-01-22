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
  
  /**
   * Link to the CTRE Pheonix Documentation
   * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___victor_s_p_x.html
   * Class privided by ctre pheonix for controlling their motor controllers
   */

  private WPI_VictorSPX frontLeft;
  private WPI_VictorSPX backLeft;

  private WPI_VictorSPX frontRight;
  private WPI_VictorSPX backRight;

  /**
   * These objects combine MotorControllers
   */
  
  private MotorControllerGroup left;
  private MotorControllerGroup right;

  /**
   * Link to WPILib for drive objects
   * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
   * Link to API
   * https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
   */
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





    // setup right drive
    frontRight = new WPI_VictorSPX(Drive.FRONT_RIGHT);
    backRight = new WPI_VictorSPX(Drive.BACK_RIGHT);

    right = new MotorControllerGroup(frontRight, backRight);





    left.setInverted(true);
    drive = new DifferentialDrive(left, right);

    setBreak();
  }

  // main meathod of the drivetrain
  public void set(double speed, double rotation)
  {
    // the last parameter asks if the inputs should be squared in this case it is set to false
    drive.arcadeDrive(speed, rotation, false);
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
