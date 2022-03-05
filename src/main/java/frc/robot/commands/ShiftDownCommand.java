// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.SHIFTER_POSITION;

public class ShiftDownCommand extends CommandBase {
  private DriveTrainSubsystem driveTrain;

  private double startTime;

  /** Creates a new ShiftUpCommand. 
   * This command runs for 0.1 seconds in coast mode, shifts the gear, and waits 0.1 seconds in coast mode
  */
  public ShiftDownCommand(DriveTrainSubsystem driveTrainSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
    startTime = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();

    driveTrain.setCoast();
    
    // zero the motor
    driveTrain.setPercent(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setPercent(0, 0);

    if (Timer.getFPGATimestamp() - startTime > 0.1)
    {
      driveTrain.setShifter(SHIFTER_POSITION.LOW);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setPercent(0, 0);
    
    driveTrain.set(new DifferentialDriveWheelSpeeds(driveTrain.getRightSpeed(), driveTrain.getLeftSpeed()));
    driveTrain.setBreak();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 0.2;
  }
}
