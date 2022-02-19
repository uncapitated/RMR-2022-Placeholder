// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import lombok.Getter;

import frc.robot.Constants.Climber;

public class ClimberSubstystem extends SubsystemBase {
  public enum CLIMBER_STATE {UP, ANGLED};

  // declare SparkMax motor
  private CANSparkMax winch;

  // winch solenoid
  private DoubleSolenoid climberSolenoid;
 
  /** Creates a new Winch. */
  public ClimberSubstystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    winch = new CANSparkMax(Climber.WINCH_MOTOR, MotorType.kBrushless);
    winch.restoreFactoryDefaults();

    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Climber.SOLENOID_OUT, Climber.SOLENOID_IN);

    // climber starts angled
    setClimberState(CLIMBER_STATE.ANGLED);
  }
  
  // set the Climber to be in or out


  //set speed
  public void set(double s)
  {
    winch.set(s);
  }

  public void setClimberState(CLIMBER_STATE state)
  {
    if (state == CLIMBER_STATE.UP)
    {
      climberSolenoid.set(Value.kForward);
    }
    else if (state == CLIMBER_STATE.ANGLED)
    {
      climberSolenoid.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
