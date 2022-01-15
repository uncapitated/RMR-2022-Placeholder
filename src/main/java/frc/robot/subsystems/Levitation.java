// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Levitation extends SubsystemBase {
  private WPI_TalonSRX levitationTalonSRX;
  private double speed;
  private DigitalInput top;
  private DigitalInput bottom;
  /** Creates a new Levitation. */
  public Levitation() {
    levitationTalonSRX = new WPI_TalonSRX(Constants.Grabber.ELEVATOR);
    top = new DigitalInput(Constants.LimitSwitches.TOP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    speed = d;
    levitationTalonSRX.set(d);
    levitationTalonSRX.setInverted(true);
  }

  public double get_levitation()
  {
    return speed;
  }
}
