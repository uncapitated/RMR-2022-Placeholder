// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.Esc;

public class Escalator extends SubsystemBase {
  //declare SparkMax motor
  private CANSparkMax sparkMax;
 
  /** Creates a new Escalator. */
  public Escalator() {
    // Use addRequirements() here to declare subsystem dependencies.
    sparkMax = new CANSparkMax(Esc.deviceId, MotorType.kBrushless);
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
