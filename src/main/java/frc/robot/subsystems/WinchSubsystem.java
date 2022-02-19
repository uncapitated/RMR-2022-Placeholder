// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import lombok.Getter;

import frc.robot.Constants.Winch;

public class WinchSubsystem extends SubsystemBase {
  // declare SparkMax motor
  private CANSparkMax sparkMax;

  // winch solenoid
  private DoubleSolenoid winchSolenoid;

  //DigitalInputs for Limit/Hall Effect 
  @Getter
  private DigitalInput top, middle, bottom;
 
  /** Creates a new Winch. */
  public WinchSubsystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    sparkMax = new CANSparkMax(Winch.WINCH_MOTOR, MotorType.kBrushless);

    winchSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Winch.SOLENOID_IN, Winch.SOLENOID_OUT);

    top = new DigitalInput(Winch.TOP_LIMIT_WINCH);
    middle = new DigitalInput(Winch.MIDDLE_LIMIT_WINCH);
    bottom = new DigitalInput(Winch.BOTTOM_LIMIT_WINCH);
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
