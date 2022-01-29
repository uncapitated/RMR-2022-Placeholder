// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmWheels extends SubsystemBase {
  private WPI_VictorSPX wheelsSRXLeft;
  private WPI_VictorSPX wheelsSRXRight;

  /** Creates a new ArmWheels. */
  public ArmWheels() {
    wheelsSRXLeft = new WPI_VictorSPX(Constants.Grabber.LEFT);
    wheelsSRXRight = new WPI_VictorSPX(Constants.Grabber.RIGHT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    wheelsSRXLeft.set(d);
    wheelsSRXRight.set(-d);
  }
}
