// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CompressorSubsystem extends SubsystemBase {
  private static Compressor compressor;

  /** Creates a new Compressor. */
  public CompressorSubsystem() {
    compressor = new Compressor(Constants.Pneumatics.COMPRESSOR_CAN_ID, PneumaticsModuleType.REVPH);

    // note that the compressor already enabled by default

    compressor.enableDigital();

    disableCompressor();
  }

  /**
   * disable compressor
   */
  public void disableCompressor()
  {
    compressor.disable();
  }

  /**
   * enable compressor
   */
  public void enableCompressor()
  {
    // enables the compressor and uses the pressure switch to turn compressor on and off
    compressor.enableDigital();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("pressure switch", compressor.getPressureSwitchValue());
  }
}
