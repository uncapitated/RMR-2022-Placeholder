// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Constants.Belt;

public class BeltSubsystem extends SubsystemBase {
  //declare SparkMax motor
  private CANSparkMax lowBelt;
  private CANSparkMax highBelt;

  private DoubleSolenoid gateSolenoid;

  private double lastUpdateTime;
  private boolean isStopped;
 
  /** Creates a new Escalator. */
  public BeltSubsystem() {
      // Use addRequirements() here to declare subsystem dependencies.
      lowBelt = new CANSparkMax(Belt.TOP_MOTOR_ID, MotorType.kBrushless);
      lowBelt.restoreFactoryDefaults();

      lowBelt.set(0);

      highBelt = new CANSparkMax(Belt.BOTTOM_MOTOR_ID, MotorType.kBrushless);
      highBelt.restoreFactoryDefaults();
      highBelt.setInverted(true);

      highBelt.set(0);

      gateSolenoid = new DoubleSolenoid(Constants.Pneumatics.COMPRESSOR_CAN_ID, PneumaticsModuleType.REVPH, Belt.SOLENOID_OUT, Belt.SOLENOID_IN);
      gateSolenoid.set(Value.kReverse);

      isStopped = true;
      lastUpdateTime = Timer.getFPGATimestamp();

  }

  public boolean getIsStopped()
  {
    return isStopped;
  }

  // intake balls
  public void intake() {
    gateSolenoid.set(Value.kForward);

    lowBelt.set(0.75);
    highBelt.set(0.75);

    lastUpdateTime = Timer.getFPGATimestamp();
  }

  public void dispense() {
    gateSolenoid.set(Value.kReverse);

    lowBelt.set(0.9);
    highBelt.set(0.9);

    lastUpdateTime = Timer.getFPGATimestamp();
  }

  public void stop() {
    lowBelt.set(0);
    highBelt.set(0);

    isStopped = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if the lastUpdateTime is 0.5 seconds ago then kill the motors
    if (!isStopped && lastUpdateTime + 0.5 < Timer.getFPGATimestamp())
    {
      stop();

      System.out.println("Belt Stopped. Please Command Subsystem!!!");
    }
  }
}
