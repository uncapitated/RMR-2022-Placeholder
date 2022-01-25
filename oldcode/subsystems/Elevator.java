// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private WPI_TalonSRX ElevatorMotor;
  private DigitalInput top;
  private DigitalInput bottom;

  /** Creates a new Elevator. */
  public Elevator() {
    ElevatorMotor = new WPI_TalonSRX(Constants.Grabber.ELEVATOR);
    top = new DigitalInput(Constants.LimitSwitches.TOP);
    bottom = new DigitalInput(Constants.LimitSwitches.BOTTOM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double d) {
    
    ElevatorMotor.set(d);
    ElevatorMotor.setInverted(true);
  }
}
