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
import frc.robot.sensors.ClimberSensorCollection;
import frc.robot.sensors.DistanceSensor;
import frc.robot.sensors.LimitSwitchSensor;

public class ClimberSubsystem extends SubsystemBase {
  public enum CLIMBER_STATE {OUT, IN};

  // declare SparkMax motor
  private CANSparkMax winch;

  // use closed loop position control
  private SparkMaxPIDController winchPID;

  // used to tell when the command reaches the setPoint
  double setPoint;


  // winch solenoid
  private DoubleSolenoid climberSolenoid;
  private CLIMBER_STATE currentClimberState;

  // sensors
  private ClimberSensorCollection sensorCollection;
  private DistanceSensor distanceSensor;

  private LimitSwitchSensor topLimitSwitchSensor;
  private LimitSwitchSensor bottomLimitSwitchSensor;

  // shuffleboard
  private ShuffleboardTab climberTab;
  private NetworkTableEntry positionEntry;
  private NetworkTableEntry currentEntry;
  private NetworkTableEntry topLimitEntry;
  private NetworkTableEntry bottomLimitEntry;
  private NetworkTableEntry distanceSensorEntry;
 
  /** Creates a new Winch. */
  public ClimberSubsystem(ClimberSensorCollection climberSensorCollection) {
    // Use addRequirements() here to declare subsystem dependencies.
    winch = new CANSparkMax(Climber.WINCH_MOTOR, MotorType.kBrushless);

    sensorCollection = climberSensorCollection;

    distanceSensor = sensorCollection.getDistanceSensor();
    topLimitSwitchSensor = sensorCollection.getTopLimitSwitchSensor();
    bottomLimitSwitchSensor = sensorCollection.getBottomLimitSwitchSensor();

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    winch.restoreFactoryDefaults();
    //winch.getEncoder().setPositionConversionFactor(Climber.WINCH_RATIO);
    //winch.getEncoder().setVelocityConversionFactor(Climber.WINCH_RATIO);

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
    setClimberState(CLIMBER_STATE.IN);

    // initialize the starting position
    setPoint = Climber.STARTING_POSITION;
    winch.getEncoder().setPosition(Climber.STARTING_POSITION);

    // setup shuffle board
    climberTab = Shuffleboard.getTab("Climber");
    positionEntry = climberTab.add("Elevator Position", winch.getEncoder().getPosition()).getEntry();
    currentEntry = climberTab.add("Winch Current", winch.getOutputCurrent()).getEntry();
    if(topLimitSwitchSensor != null) topLimitEntry = climberTab.add("Top Limit Switch", topLimitSwitchSensor.isPressed()).getEntry();
    if(bottomLimitSwitchSensor != null) bottomLimitEntry = climberTab.add("Bottom Limit Switch", bottomLimitSwitchSensor.isPressed()).getEntry();
    if(distanceSensor != null) distanceSensorEntry = climberTab.add("REV Distance Sensor", distanceSensor.getDistance()).getEntry();
  }

  public void setPosition(double position) {
    switch(currentClimberState)
    {
      case IN:
      // clamp the highest it can go
      position = Math.min(position, Climber.MAX_IN);
      // clamp the lowest it can go
      position = Math.max(position, Climber.MIN_IN);
      break;

      case OUT:
      // clamp the highest it can go
      position = Math.min(position, Climber.MAX_OUT);
      // clamp the lowest it can go
      position = Math.max(position, Climber.MIN_OUT);
      break;
    }

    // ues pid to run motor
    winchPID.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    setPoint = position;

    if (topLimitSwitchSensor.isPressed()) // top sensor is at MAX_ANGLED
    {
      winch.getEncoder().setPosition(Climber.MAX_IN + 0.02);
    }

    if (bottomLimitSwitchSensor.isPressed()) // bottom sensor is at MIN_UP
    {
      winch.getEncoder().setPosition(Climber.MIN_OUT - 0.02);
    }

    // guess that negative current is up (Gearboxes?)
    winch.get();
    if(winch.getOutputCurrent() < 0 && topLimitSwitchSensor.isPressed())
    {
      winch.set(0);
    }
    // guess that positive current is down
    if(winch.getOutputCurrent() > 0 && bottomLimitSwitchSensor.isPressed())
    {
      winch.set(0);
    }
  }

  /**
   * sets the elevator to the highest it can go
   */
  public void setPositionTop()
  {
    if(currentClimberState == CLIMBER_STATE.IN) {
      setPosition(Climber.MAX_IN);
    }
    else if (currentClimberState == CLIMBER_STATE.OUT) {
      setPosition(Climber.MAX_OUT);
    }
  }

  /**
   * sets the elevator to a point where it can change from UP to ANGLED
   */
  public void setPositionTransition()
  {
    // go to a mid way point
    double midwayPoint = (Climber.MIN_IN + Climber.MAX_OUT) / 2;
    setPosition(midwayPoint);
  }

  public void setPositionBottom()
  {
    if(currentClimberState == CLIMBER_STATE.IN) {
      setPosition(Climber.MIN_IN);
    }
    else if (currentClimberState == CLIMBER_STATE.OUT) {
      setPosition(Climber.MIN_OUT);
    }
  }

  /**
   * @return an estimated elevator position
   */
  public double getElevatorPosition()
  {
    return winch.getEncoder().getPosition();
  }

  public void set(double val) {
    winch.set(val);
    checkLimitSwitches();
  }
  
  public CLIMBER_STATE getClimberState() {
    return currentClimberState;
  }

  // set the Climber to be in or out
  public void setClimberState(CLIMBER_STATE state) {
    currentClimberState = state;

    if (state == CLIMBER_STATE.OUT) {
      climberSolenoid.set(Value.kForward);
    }
    else if (state == CLIMBER_STATE.IN) {
      climberSolenoid.set(Value.kReverse);
    }
  }

  /** helper function for setClimberState */
  public void setClimberIn() {
    setClimberState(CLIMBER_STATE.IN);
  }
  /** helper function for setClimberState */
  public void setClimberOut() {
    setClimberState(CLIMBER_STATE.OUT);
  }

  public boolean atSetPoint() {

    // if the winch is within 5 cm of the set point
    return (Math.abs(winch.getEncoder().getPosition() - setPoint) < 0.05);
  }

  public void checkLimitSwitches() {
    // guess that negative current is up (Gearboxes?)
    if(winch.get() > 0 && topLimitSwitchSensor.isPressed())
    {
      winch.set(0);
    }
    // guess that positive current is down
    if(winch.get() < 0 && bottomLimitSwitchSensor.isPressed())
    {
      winch.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update the PID Controller

    checkLimitSwitches();

    // update encoder position if switches are pressed
    if (topLimitSwitchSensor.isPressed()) // top sensor is at MAX_ANGLED
    {
      winch.getEncoder().setPosition(Climber.MAX_IN + 0.02);
    }

    if (bottomLimitSwitchSensor.isPressed()) // bottom sensor is at MIN_UP
    {
      winch.getEncoder().setPosition(Climber.MIN_OUT - 0.02);
    }

    // update shuffleboard
    positionEntry.setDouble(winch.getEncoder().getPosition());
    currentEntry.setDouble(winch.getOutputCurrent());
    // could be null if in simulation
    if(topLimitSwitchSensor != null) topLimitEntry.setBoolean(topLimitSwitchSensor.isPressed());
    if(bottomLimitSwitchSensor != null) bottomLimitEntry.setBoolean(bottomLimitSwitchSensor.isPressed());
    if(distanceSensor != null) distanceSensorEntry.setDouble(distanceSensor.getDistance());
  }
}
