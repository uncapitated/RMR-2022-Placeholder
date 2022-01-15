// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Levitation extends SubsystemBase {
  private WPI_TalonSRX levitationTalonSRX;
  /** Creates a new Levitation. */
  public Levitation() {
    levitationTalonSRX = new WPI_TalonSRX(Constants.Grabber.ELEVATOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    levitationTalonSRX.set(d);
    levitationTalonSRX.setInverted(true);
  }
}
