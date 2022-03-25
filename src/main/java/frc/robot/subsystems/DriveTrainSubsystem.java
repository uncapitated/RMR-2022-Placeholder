// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Drive;
import frc.robot.simulation.PhysicsSim;
import frc.robot.simulation.Simulation;
import lombok.Getter;


public class DriveTrainSubsystem extends SubsystemBase {
  
  // class for getting robot position
  private DifferentialDriveOdometry odometry;
  /** Distance the left side of the robot has traveled */
  private double leftPosition;
  /** Distance the right side of the robot has traveled */
  private double rightPosition;

  private double lastLeftEncoderPosition;
  private double lastRightEncoderPosition;
  private double lastUpdateTime;

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

  // Simulation
  // Simulation object
  private Simulation sim;

  // shifter
  private DoubleSolenoid vent;
  public enum VENT_POSITION {CLOSED, OPEN}
  @Getter
  private VENT_POSITION ventPosition;

  public void closeVent(){
    ventPosition = VENT_POSITION.CLOSED;
    vent.set(Value.kReverse);
  }

  public void openVent(){
    ventPosition = VENT_POSITION.OPEN;
    vent.set(Value.kForward);
  }

  // rotation of drive - should be changed to rely on Gyro
  private Rotation2d rotation;

  // timeout
  private boolean isStopped;
  private double safetyTimeout;

  // shuffleboard
  ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Train");
  NetworkTableEntry leftSteps = driveTab.add("Left Motor", 0).getEntry();
  NetworkTableEntry rightSteps = driveTab.add("Right Motor", 0).getEntry();


  
  

  /** Creates a new DriveTrain. */
  public DriveTrainSubsystem(){
    // used to calculate robot position
    rotation = new Rotation2d(0, 0);
    odometry = new DifferentialDriveOdometry(rotation, new Pose2d());

    leftPosition = 0;
    rightPosition = 0;

    lastUpdateTime = Timer.getFPGATimestamp();

    // setup left drive
    frontLeft = new WPI_TalonFX(Drive.FRONT_LEFT);
    backLeft = new WPI_TalonFX(Drive.BACK_LEFT); 
    frontLeft.setNeutralMode(NeutralMode.Coast);
    backLeft.setNeutralMode(NeutralMode.Coast);

    backLeft.follow(frontLeft);

    // setup right drive
    frontRight = new WPI_TalonFX(Drive.FRONT_RIGHT);
    backRight = new WPI_TalonFX(Drive.BACK_RIGHT);
    frontRight.setNeutralMode(NeutralMode.Coast);
    backRight.setNeutralMode(NeutralMode.Coast);

    backRight.follow(frontRight);

    backRight.setInverted(true);
    frontRight.setInverted(true);


    // reset motors
    configTalon(frontLeft);
    configTalon(frontRight);
    configTalon(backLeft);
    configTalon(backRight);

    // make sure that the back motors follow the front motors
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    lastLeftEncoderPosition = frontLeft.getSelectedSensorPosition();
    lastRightEncoderPosition = frontRight.getSelectedSensorPosition();

    // setup vent
    vent = new DoubleSolenoid(Constants.Pneumatics.COMPRESSOR_CAN_ID, PneumaticsModuleType.REVPH, Constants.Drive.VENT_HIGH, Constants.Drive.VENT_LOW);
    closeVent();
    

    // setup timeout
    isStopped = true;
    safetyTimeout = Timer.getFPGATimestamp();

    // easy to push robot
    setCoast();
    setPercent(0, 0);
  }

  public DriveTrainSubsystem(Simulation sim)
  {
    this();
    if (RobotBase.isSimulation())
    {
      this.sim = sim;

      // add physics sim
      PhysicsSim.getInstance().addTalonFX(frontLeft, 0.75, 20660);
      PhysicsSim.getInstance().addTalonFX(frontRight, 0.75, 20660);
      PhysicsSim.getInstance().addTalonFX(backLeft, 0.75, 20660);
      PhysicsSim.getInstance().addTalonFX(backRight, 0.75, 20660);
    }
  }

  public void stop()
  {
    setPercent(0, 0);

    isStopped = true;
  }

  /**
   * set with chassis speeds m/s, m/s, rad/s
   */
  public void set(ChassisSpeeds chassisSpeeds)
  {
    chassisSpeeds.vyMetersPerSecond = 0;
    chassisSpeeds.omegaRadiansPerSecond = -chassisSpeeds.omegaRadiansPerSecond;
    // convert to wheel speeds
    set(Drive.KINEMATICS.toWheelSpeeds(chassisSpeeds));
  }

  /**
   *  set drivetrain with wheel speeds
   */
  public void set(DifferentialDriveWheelSpeeds wheelSpeeds)
  {
    
    safetyTimeout = Timer.getFPGATimestamp();

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
   * 
   * @param leftPercent the percent output of the left side of the drive train
   * @param rightPercent the percent output of the right side of the drive train
   */
  public void setPercent(double leftPercent, double rightPercent)
  {
    // reset the timer
    safetyTimeout = Timer.getFPGATimestamp();

    frontLeft.set(TalonFXControlMode.PercentOutput, leftPercent);
    frontRight.set(TalonFXControlMode.PercentOutput, rightPercent);
  }

  public double getRightSpeed()
  {
    return toRobotSpeed(frontRight.getSelectedSensorVelocity());
  }
  public double getLeftSpeed()
  {
    return toRobotSpeed(frontLeft.getSelectedSensorVelocity());
  }


  // set shifter
  // public void setShifter(SHIFTER_POSITION position)
  // {
  //   shifterPosition = position;

  //   if(position == SHIFTER_POSITION.HIGH)
  //   {
  //     shifter.set(Value.kReverse);
  //   }
  //   else if (position == SHIFTER_POSITION.LOW)
  //   {
  //     shifter.set(Value.kForward);
  //   }
  // }

  // public SHIFTER_POSITION getShifter()
  // {
  //   return shifterPosition;
  // }

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



  public void setPosition(Pose2d robotPos)
  {
    rotation = robotPos.getRotation();
    odometry.resetPosition(robotPos, rotation);

    leftPosition = 0;
    rightPosition = 0;
  }

  public Pose2d getCalculatedRobotPose()
  {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // check for timeouts
    // if the lastUpdateTime is 0.5 seconds ago then kill the motors
    if (!isStopped && safetyTimeout + 0.5 < Timer.getFPGATimestamp())
    {
      stop();

      System.out.println("Drive Stopped. 0.5 second timeout exceeded Please Command Subsystem!!!");
    }

    

    // update odometry
    double changeInTime = Timer.getFPGATimestamp() - lastUpdateTime;
    double changeInLeftPosition = toRobotPosition(frontLeft.getSelectedSensorPosition() - lastLeftEncoderPosition);
    double changeInRightPosition = toRobotPosition(frontRight.getSelectedSensorPosition() - lastRightEncoderPosition);

    double leftWheelSpeed = changeInLeftPosition / changeInTime;
    double rightWheelSpeed = changeInRightPosition / changeInTime;

    leftPosition += changeInLeftPosition;
    rightPosition += changeInRightPosition;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
    ChassisSpeeds chassisSpeeds = Constants.Drive.KINEMATICS.toChassisSpeeds(wheelSpeeds);
    rotation = rotation.rotateBy(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(Robot.period));

    odometry.update(rotation, leftPosition, rightPosition);

    // reset for next loop
    lastLeftEncoderPosition = frontLeft.getSelectedSensorPosition();
    lastRightEncoderPosition = frontRight.getSelectedSensorPosition();
    lastUpdateTime = Timer.getFPGATimestamp();
  }

  // code for simulation (does not run when this isn't a simulation)
  @Override
  public void simulationPeriodic()
  {
    PhysicsSim.getInstance().run();

    // update sim
    sim.updateRobotPos(odometry.getPoseMeters());
  }

  private void configTalon(WPI_TalonFX talon)
  {
    /* Factory Default all hardware to prevent unexpected behaviour */
		talon.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		talon.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.DrivePID.kPIDLoopIdx, 
											Constants.DrivePID.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		talon.configNominalOutputForward(0, Constants.DrivePID.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.DrivePID.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.DrivePID.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.DrivePID.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		talon.config_kF(Constants.DrivePID.kPIDLoopIdx, Constants.DrivePID.kGains_Velocity.kF, Constants.DrivePID.kTimeoutMs);
		talon.config_kP(Constants.DrivePID.kPIDLoopIdx, Constants.DrivePID.kGains_Velocity.kP, Constants.DrivePID.kTimeoutMs);
		talon.config_kI(Constants.DrivePID.kPIDLoopIdx, Constants.DrivePID.kGains_Velocity.kI, Constants.DrivePID.kTimeoutMs);
		talon.config_kD(Constants.DrivePID.kPIDLoopIdx, Constants.DrivePID.kGains_Velocity.kD, Constants.DrivePID.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);
  }



  private double toRobotSpeed(double motorSpeed)
  {
    // In rotations per second
    motorSpeed *= Drive.DRIVE_VELOCITY_FACTOR;

    // In rotations per second
    if (false) {
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
    motorPosition /= Drive.DRIVE_SPR;

    // In rotations
    if (false) {
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
    if (false) {
      robotSpeed /= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      robotSpeed /= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In rotations per second
    robotSpeed /= Drive.DRIVE_VELOCITY_FACTOR;

    return robotSpeed;
  }

  private double toMotorPosition(double robotPosition)
  {
    // In Meters
    robotPosition /= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    // In rotations
    if (false) {
      robotPosition /= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      robotPosition /= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In steps
    robotPosition *= Drive.DRIVE_VELOCITY_FACTOR;

    return robotPosition;
  }
}