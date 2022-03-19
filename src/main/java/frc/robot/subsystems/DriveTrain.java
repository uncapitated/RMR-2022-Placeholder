// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Autonomous;
import frc.robot.Constants;
import frc.robot.Constants.Drive;

public class DriveTrain extends SubsystemBase {
  
  /**
   * Link to the CTRE Phoenix Documentation
   * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___victor_s_p_x.html
   * Class provided by CTRE Phoenix  for controlling their motor controllers
   */
   
  private DifferentialDriveOdometry odometry;

  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;

  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;

  private DoubleSolenoid shifter;
  private enum ShifterPosition {LOW, HIGH}
  private ShifterPosition shifterPosition;

  /**
   * These objects combine MotorControllers
   */
  
  private MotorControllerGroup left;
  private MotorControllerGroup right;

  private double leftPos = 0;
  private double rightPos = 0;

  private Rotation2d rotation;

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
    rotation = Autonomous.getAutonomous().getStartingRotation();
    odometry = new DifferentialDriveOdometry(rotation, Autonomous.getAutonomous().getStartingPos());

    // setup left drive
    frontLeft = new WPI_TalonFX(Drive.FRONT_LEFT);
    backLeft = new WPI_TalonFX(Drive.BACK_LEFT); 

    // setup right drive
    frontRight = new WPI_TalonFX(Drive.FRONT_RIGHT);
    backRight = new WPI_TalonFX(Drive.BACK_RIGHT);


    // create motor groups
    left = new MotorControllerGroup(frontLeft, backLeft);
    right = new MotorControllerGroup(frontRight, backRight);

    left.setInverted(true);

    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);

    shifterPosition = ShifterPosition.LOW;
    shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shifter.LOW, Constants.Shifter.HIGH);
    shifter.set(Value.kForward);

    drive = new DifferentialDrive(left, right);

    setBreak();
  }

  // main method of the DriveTrain
  public void set(double speed, double rotation)
  {
    // the last parameter asks if the inputs should be squared in this case it is set to false
    drive.arcadeDrive(speed, rotation, false);
  }

  public void setLeftAndRight(double leftSpeed, double rightSpeed)
  {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * sets the motors to coast mode to prevent current usage
   */
  public void setCoast()
  {
    TalonFX[] motors = {frontLeft, frontRight, backLeft, backRight};
    for (TalonFX motor : motors) {
      motor.setNeutralMode(NeutralMode.Coast);
    }

    set(0, 0);
  }

  /**
   * sets the motors to break mode to enable better control
   */
  public void setBreak()
  {
    TalonFX[] motors = {frontLeft, frontRight, backLeft, backRight};
    for (TalonFX motor : motors) {
      motor.setNeutralMode(NeutralMode.Brake);
    }
  }

  public DifferentialDrive getDrive() {
    return drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double leftWheelSpeed = (frontLeft.getSelectedSensorVelocity(1) * backLeft.getSelectedSensorVelocity(1)) / 2; 
    double rightWheelSpeed = (frontRight.getSelectedSensorVelocity(1) * backRight.getSelectedSensorPosition(1)) / 2;

    // In rotations per second
    leftWheelSpeed *= Constants.Motor.DRIVE_VELOCITY_FACTOR;
    rightWheelSpeed *= Constants.Motor.DRIVE_VELOCITY_FACTOR;

    // In rotations per second
    if (shifterPosition == ShifterPosition.LOW) {
      leftWheelSpeed *= Constants.Drive.HIGH_GEAR_RATIO;
      rightWheelSpeed *= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      leftWheelSpeed *= Constants.Drive.LOW_GEAR_RATIO;
      rightWheelSpeed *= Constants.Drive.LOW_GEAR_RATIO;
    }

    leftWheelSpeed *= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;
    rightWheelSpeed *= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
  }
}