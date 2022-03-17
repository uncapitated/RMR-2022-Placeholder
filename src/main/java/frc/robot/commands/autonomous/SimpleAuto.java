// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutonomousPoints;
import frc.robot.commands.BeltDispenseCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAuto extends SequentialCommandGroup {

  DriveTrainSubsystem driveTrain;
  AutonomousPoints points;

  /** Creates a new SimpleAuto. */
  public SimpleAuto(DriveTrainSubsystem driveTrainSubsystem, BeltSubsystem beltSubsystem, AutonomousPoints autoPoints) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    driveTrain = driveTrainSubsystem;
    points = autoPoints;

    // go to the hub
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(1)));

    // dispense ball for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltDispenseCommand(beltSubsystem),
        new WaitCommand(0.5)
      )
    );

    // exit the inner terminal area going reversed
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(2), true));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setPosition(points.getStartingPosition());

    super.initialize();
  }
}
