// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import lombok.Getter;

import frc.robot.Constants.winch;;

public class Winch extends SubsystemBase {
  //declare SparkMax motor
  private CANSparkMax sparkMax;

  //DigitalInputs for Limit/Hall Effect 
  @Getter
  private DigitalInput top, middle, bottom;
 
  /** Creates a new Winch. */
  public Winch() {
    // Use addRequirements() here to declare subsystem dependencies.
    sparkMax = new CANSparkMax(winch.WINCH_MOTOR, MotorType.kBrushless);
    top = new DigitalInput(winch.TOP_LIMIT_WINCH);
    middle = new DigitalInput(winch.MIDDLE_LIMIT_WINCH);
    bottom = new DigitalInput(winch.BOTTOM_LIMIT_WINCH);
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
