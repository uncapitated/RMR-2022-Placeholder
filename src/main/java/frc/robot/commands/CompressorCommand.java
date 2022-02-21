// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CompressorConstants;
import frc.robot.subsystems.CompressorSubsystem;

public class CompressorCommand extends CommandBase {
  // subsystem for enabling and disabling the compressor
  private CompressorSubsystem compressorSubsystem;

  // switch for controlling the compressor
  private DigitalInput compressorSwitch;

  /** Creates a new CompressorCommand. */
  public CompressorCommand(CompressorSubsystem compressorSubsystem) {
    this.compressorSubsystem = compressorSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(compressorSubsystem);

    compressorSwitch = new DigitalInput(CompressorConstants.COMPRESSOR_SWITCH);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    compressorSubsystem.disableCompressor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Compressor", compressorSwitch.get());

    boolean switchEnabled = !compressorSwitch.get();
    if (switchEnabled)
    {
      SmartDashboard.putBoolean("Test", true);
      compressorSubsystem.enableCompressor();
    }
    else
    {
      SmartDashboard.putBoolean("Test", false);
      compressorSubsystem.disableCompressor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
