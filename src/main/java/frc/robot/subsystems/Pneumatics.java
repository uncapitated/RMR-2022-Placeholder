// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid pneumatic;
  public Pneumatics() {
    pneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Shifter.HIGH, Constants.Shifter.LOW);
    /** Creates a new Pneumatics. */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shiftHigh()
  {
    pneumatic.set(Value.kForward);
  }

  public void shiftLow()
  {
    pneumatic.set(Value.kReverse);
  }
}
