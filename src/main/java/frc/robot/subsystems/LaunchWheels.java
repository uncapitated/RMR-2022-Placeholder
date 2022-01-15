// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaunchWheels extends SubsystemBase {
  private WPI_TalonSRX wheelsSRXLeft;
  private WPI_TalonSRX wheelsSRXRight;

  /** Creates a new LaunchWheels. */
  public LaunchWheels() {
    wheelsSRXLeft = new WPI_TalonSRX(Constants.Grabber.LEFT);
    wheelsSRXRight = new WPI_TalonSRX(Constants.Grabber.RIGHT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    wheelsSRXLeft.set(d);
    wheelsSRXRight.set(-d);
    if (Math.abs(d) > 0.01){
      System.out.println("Triggered");
    }
  }
}
