// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
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
import frc.robot.Constants.Drive;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.Simulation;;

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

  // Simulation
  // Simulation object
  private Simulation sim;


  // shifter
  private DoubleSolenoid shifter;
  private enum ShifterPosition {LOW, HIGH}
  private ShifterPosition shifterPosition;

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

    // setup shifter
    shifterPosition = ShifterPosition.LOW;
    shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shifter.LOW, Constants.Shifter.HIGH);
    shifter.set(Value.kForward);

    setBreak();
  }

  public DriveTrain(Simulation sim)
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
    double leftWheelSpeed = (frontLeft.getSelectedSensorVelocity() + backLeft.getSelectedSensorVelocity()) / 2; 
    double rightWheelSpeed = (frontRight.getSelectedSensorVelocity() + backRight.getSelectedSensorVelocity()) / 2;
    // anticipating encoder positions desyncing
    double leftEncoder = frontLeft.getSelectedSensorPosition();
    double rightEncoder = frontRight.getSelectedSensorPosition();

    leftWheelSpeed = toRobotSpeed(leftWheelSpeed);
    rightWheelSpeed = toRobotSpeed(rightWheelSpeed);

    leftEncoder = toRobotPosition(leftEncoder);
    rightEncoder = toRobotPosition(rightEncoder);

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
    ChassisSpeeds chassisSpeeds = Constants.Drive.KINEMATICS.toChassisSpeeds(wheelSpeeds);
    rotation = rotation.rotateBy(new Rotation2d(chassisSpeeds.omegaRadiansPerSecond).times(Robot.period));

    odometry.update(rotation, leftEncoder, rightEncoder);
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
                                            Constants.PID.kPIDLoopIdx, 
											Constants.PID.kTimeoutMs);
											

		/* Config the peak and nominal outputs */
		talon.configNominalOutputForward(0, Constants.PID.kTimeoutMs);
		talon.configNominalOutputReverse(0, Constants.PID.kTimeoutMs);
		talon.configPeakOutputForward(1, Constants.PID.kTimeoutMs);
		talon.configPeakOutputReverse(-1, Constants.PID.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		talon.config_kF(Constants.PID.kPIDLoopIdx, Constants.PID.kGains_Velocity.kF, Constants.PID.kTimeoutMs);
		talon.config_kP(Constants.PID.kPIDLoopIdx, Constants.PID.kGains_Velocity.kP, Constants.PID.kTimeoutMs);
		talon.config_kI(Constants.PID.kPIDLoopIdx, Constants.PID.kGains_Velocity.kI, Constants.PID.kTimeoutMs);
		talon.config_kD(Constants.PID.kPIDLoopIdx, Constants.PID.kGains_Velocity.kD, Constants.PID.kTimeoutMs);
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
    motorPosition /= Constants.Motor.DRIVE_SPR;

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
    // In Meters
    robotPosition /= Math.PI * Constants.Drive.WHEEL_RADIUS * 2;

    // In rotations
    if (shifterPosition == ShifterPosition.HIGH) {
      robotPosition /= Constants.Drive.HIGH_GEAR_RATIO;
    } else {
      robotPosition /= Constants.Drive.LOW_GEAR_RATIO;
    }

    // In steps
    robotPosition *= Constants.Motor.DRIVE_VELOCITY_FACTOR;

    return robotPosition;
  }
}