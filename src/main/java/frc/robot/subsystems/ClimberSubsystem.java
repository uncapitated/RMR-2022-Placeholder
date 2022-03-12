// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.sensors.DistanceSensor;
import frc.robot.sensors.LimitSwitchSensor;

public class ClimberSubsystem extends SubsystemBase {
  public enum CLIMBER_STATE {UP, ANGLED};

  // declare SparkMax motor
  private CANSparkMax winch;

  // use closed loop position control
  private SparkMaxPIDController winchPID;
  /** setPoint */
  private double setPoint;


  // winch solenoid
  private DoubleSolenoid climberSolenoid;
  private CLIMBER_STATE currentClimberState;

  private ShuffleboardTab climberTab;
  private NetworkTableEntry positionEntry;

  private DistanceSensor distanceSensor;

  private LimitSwitchSensor topLimitSwitchSensor;
  private LimitSwitchSensor bottomLimitSwitchSensor;
 
  /** Creates a new Winch. */
  public ClimberSubsystem(DistanceSensor distanceSensor, LimitSwitchSensor topLimitSwitchSensor, LimitSwitchSensor bottomLimitSwitchSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    winch = new CANSparkMax(Climber.WINCH_MOTOR, MotorType.kBrushless);

    this.distanceSensor = distanceSensor;
    this.topLimitSwitchSensor = topLimitSwitchSensor;
    this.bottomLimitSwitchSensor = bottomLimitSwitchSensor;

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    winch.restoreFactoryDefaults();

    // setup PID
    winchPID = winch.getPIDController();

    // set PID coefficients
    winchPID.setP(Climber.kGains_Position.kP);
    winchPID.setI(Climber.kGains_Position.kI);
    winchPID.setD(Climber.kGains_Position.kD);
    winchPID.setIZone(Climber.kGains_Position.kIzone);
    winchPID.setFF(Climber.kGains_Position.kF);
    winchPID.setOutputRange(-Climber.kGains_Position.kPeakOutput, Climber.kGains_Position.kPeakOutput);

    climberSolenoid = new DoubleSolenoid(Constants.Pneumatics.COMPRESSOR_CAN_ID, PneumaticsModuleType.REVPH, Climber.SOLENOID_OUT, Climber.SOLENOID_IN);

    // climber starts angled
    setClimberState(CLIMBER_STATE.ANGLED);

    // initialize the starting position
    winch.getEncoder().setPosition(toMotorRotations(Climber.STARTING_POSITION));
    setPoint = Climber.STARTING_POSITION;

    climberTab = Shuffleboard.getTab("Climber");
    positionEntry = climberTab.add("Elevator Position", winch.getEncoder().getPosition()).getEntry();
  }

  public void setElevatorPosition(double position) {
      switch(currentClimberState)
      {
        case ANGLED:
        // clamp the highest it can go
        position = Math.min(position, Climber.MAX_ANGLED);
        // clamp the lowest it can go
        position = Math.max(position, Climber.MIN_ANGLED);
        break;

        case UP:
        // clamp the highest it can go
        position = Math.min(position, Climber.MAX_UP);
        // clamp the lowest it can go
        position = Math.max(position, Climber.MIN_UP);
        break;
      }

      if (position > setPoint && !topLimitSwitchSensor.getPressed()) {
        setPoint = position;
      } else if (position < setPoint && !bottomLimitSwitchSensor.getPressed()) {
        setPoint = position;
      }
  }

  /**
   * @return an estimated elevator position
   */
  public double getElevatorPosition()
  {
    return toElevatorMeters(winch.getEncoder().getPosition());
  }

  public void set(double val){
    winch.set(val);
  }
  
  public CLIMBER_STATE getClimberState() {
    return currentClimberState;
  }

  // set the Climber to be in or out
  public void setClimberState(CLIMBER_STATE state)
  {
    currentClimberState = state;

    if (state == CLIMBER_STATE.UP)
    {
      climberSolenoid.set(Value.kForward);
    }
    else if (state == CLIMBER_STATE.ANGLED)
    {
      climberSolenoid.set(Value.kReverse);
    }
  }

  /** helper function for setClimberState */
  public void setClimberAngled()
  {
    setClimberState(CLIMBER_STATE.ANGLED);
  }
  /** helper function for setClimberState */
  public void setClimberUP()
  {
    setClimberState(CLIMBER_STATE.UP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update the PID Controller
    // winchPID.setReference(toMotorRotations(setPoint), CANSparkMax.ControlType.kPosition);

    // update shuffleboard
    positionEntry.setValue(winch.getEncoder().getPosition());
  }

  
  private double toElevatorMeters(double rotations) {
    return rotations * Climber.WINCH_RATIO;
  }

  private double toMotorRotations(double meters) {
    return meters / Climber.WINCH_RATIO;
  }
}
