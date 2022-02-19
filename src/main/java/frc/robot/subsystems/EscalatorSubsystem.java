// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.Escalator;

public class EscalatorSubsystem extends SubsystemBase {
  //declare SparkMax motor
  private CANSparkMax sparkMax;

  private DoubleSolenoid escalatorSolenoid;
 
  /** Creates a new Escalator. */
  public EscalatorSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    sparkMax = new CANSparkMax(Escalator.deviceId, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
  }

  //set speed
  public void set(double s)
  {
    sparkMax.set(s);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
