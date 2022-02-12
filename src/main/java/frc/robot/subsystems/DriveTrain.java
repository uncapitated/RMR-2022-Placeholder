// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Autonomous;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Drive;;

public class DriveTrain extends SubsystemBase {
  
  // class for getting robot position
  private DifferentialDriveOdometry odometry;

  /**
   * Link to the CTRE Phoenix Documentation
   * https://store.ctr-electronics.com/content/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_f_x.html
   * Class provided by CTRE Phoenix  for controlling their motor controllers
   */

  // motors
  private WPI_TalonFX frontLeft;
  private WPI_TalonFX backLeft;

  private WPI_TalonFX frontRight;
  private WPI_TalonFX backRight;

  // motor encoder simulation devices
  private TalonFXSimCollection frontLeftSim;
  private TalonFXSimCollection frontRightSim;

  // shifter
  private DoubleSolenoid shifter;
  private enum ShifterPosition {LOW, HIGH}
  private ShifterPosition shifterPosition;

  // position of the left and right encoders
  private double leftPos = 0;
  private double rightPos = 0;

  // rotation of drive - should be changed to rely on Gyro
  private Rotation2d rotation;

  /** Creates a new DriveTrain. */
  public DriveTrain()
  {
    rotation = Autonomous.getAutonomous().getStartingRotation();
    odometry = new DifferentialDriveOdometry(rotation, Autonomous.getAutonomous().getStartingPos());

    // setup left drive
    frontLeft = new WPI_TalonFX(Drive.FRONT_LEFT);
    backLeft = new WPI_TalonFX(Drive.BACK_LEFT); 

    backLeft.follow(frontLeft);

    // setup right drive
    frontRight = new WPI_TalonFX(Drive.FRONT_RIGHT);
    backRight = new WPI_TalonFX(Drive.BACK_RIGHT);

    backRight.follow(frontRight);

    frontLeft.setInverted(true);

    // make sure that the back motors follow the front motors
    frontRight.setSelectedSensorPosition(0);
    frontLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);

    // setup sim devices
    frontLeftSim = new TalonFXSimCollection(frontLeft);
    frontRightSim = new TalonFXSimCollection(frontRight);

    // setup shifter
    shifterPosition = ShifterPosition.LOW;
    shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shifter.LOW, Constants.Shifter.HIGH);
    shifter.set(Value.kForward);

    setBreak();
  }

  public void stop()
  {
    set(new DifferentialDriveWheelSpeeds(0, 0));
  }

  public void set(DifferentialDriveWheelSpeeds wheelSpeeds)
  {
    // convert wheelSpeeds to motor speeds
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelocity = wheelSpeeds.rightMetersPerSecond;

    // convert to motor velocity
    leftVelocity = toMotorSpeed(leftVelocity);
    rightVelocity = toMotorSpeed(rightVelocity);

    frontLeft.set(TalonFXControlMode.Velocity, leftVelocity);
    frontRight.set(TalonFXControlMode.Velocity, rightVelocity);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update odomitry
    double leftWheelSpeed = (frontLeft.getSelectedSensorVelocity(1) + backLeft.getSelectedSensorVelocity(1)) / 2; 
    double rightWheelSpeed = (frontRight.getSelectedSensorVelocity(1) + backRight.getSelectedSensorVelocity(1)) / 2;
    // anticipating encoder positions desyncing
    double leftEncoder = frontLeft.getSelectedSensorPosition(1);
    double rightEncoder = frontRight.getSelectedSensorPosition(1);

    leftWheelSpeed = toRobotSpeed(leftWheelSpeed);
    rightWheelSpeed = toRobotSpeed(rightWheelSpeed);

    leftEncoder = toRobotPosition(leftEncoder);
    rightEncoder = toRobotPosition(rightEncoder);

    double leftDistanceMeters = leftEncoder - leftPos;
    double rightDistanceMeters = rightEncoder - rightPos;

    leftPos = leftEncoder;
    rightPos = rightEncoder;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
    ChassisSpeeds chassisSpeeds = Constants.Drive.KINEMATICS.toChassisSpeeds(wheelSpeeds);
    rotation.rotateBy(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(Robot.period));

    odometry.update(rotation, leftDistanceMeters, rightDistanceMeters);
  }

  // code for simulation (does not run when this isn't a simulation)
  @Override
  public void simulationPeriodic()
  {
    double maxVelocityChange = 100.0;

    // update frontleft drive 
    if (frontLeft.getActiveTrajectoryVelocity() - frontLeft.getSelectedSensorVelocity() < maxVelocityChange)
    {
      frontLeftSim.setIntegratedSensorVelocity((int) frontLeft.getActiveTrajectoryVelocity());
    }
    else
    {
      frontLeftSim.setIntegratedSensorVelocity((int) (frontLeft.getSelectedSensorVelocity() +
        Math.copySign(maxVelocityChange, frontLeft.getActiveTrajectoryVelocity() - frontLeft.getSelectedSensorVelocity())));
    }

    // update backleft drive 
    if (frontRight.getActiveTrajectoryVelocity() - frontLeft.getSelectedSensorVelocity() < maxVelocityChange)
    {
      frontRightSim.setIntegratedSensorVelocity((int) frontLeft.getActiveTrajectoryVelocity());
    }
    else
    {
      frontRightSim.setIntegratedSensorVelocity((int) (frontLeft.getSelectedSensorVelocity() +
        Math.copySign(maxVelocityChange, frontLeft.getActiveTrajectoryVelocity() - frontLeft.getSelectedSensorVelocity())));
    }
  }

  private double toRobotSpeed(double motorSpeed)
  {
    // In rotations per second
    motorSpeed *= Constants.Motor.DRIVE_VELOCITY_FACTOR;

    // In rotations per second
    if (shifterPosition == ShifterPosition.HIGH) {
      motorSpeed *= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      motorSpeed *= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In Meters Per Second
    motorSpeed *= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    return motorSpeed;
  }

  private double toRobotPosition(double motorPosition)
  {
    // In rotations
    motorPosition *= Constants.Motor.DRIVE_SPR;

    // In rotations
    if (shifterPosition == ShifterPosition.HIGH) {
      motorPosition *= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      motorPosition *= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In positions
    motorPosition *= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    return motorPosition;
  }

  private double toMotorSpeed(double robotSpeed)
  {
    // In Meters Per Second
    robotSpeed /= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    // In rotations per second
    if (shifterPosition == ShifterPosition.HIGH) {
      robotSpeed /= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      robotSpeed /= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In rotations per second
    robotSpeed /= Constants.Motor.DRIVE_VELOCITY_FACTOR;

    return robotSpeed;
  }

  private double toMotorPosition(double robotPosition)
  {
    // In Meters Per Second
    robotPosition /= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    // In rotations per second
    if (shifterPosition == ShifterPosition.HIGH) {
      robotPosition /= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      robotPosition /= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In rotations per second
    robotPosition /= Constants.Motor.DRIVE_VELOCITY_FACTOR;

    return robotPosition;
  }
}