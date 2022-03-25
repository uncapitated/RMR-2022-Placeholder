// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutonomousPoints;
import frc.robot.commands.BeltDispenseCommand;
import frc.robot.commands.BeltIntakeCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleAutoCommand extends SequentialCommandGroup {

  DriveTrainSubsystem driveTrain;
  AutonomousPoints points;

  /** Creates a new SimpleAuto. */
  public SimpleAutoCommand(DriveTrainSubsystem driveTrainSubsystem, BeltSubsystem beltSubsystem, AutonomousPoints autoPoints) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    driveTrain = driveTrainSubsystem;
    points = autoPoints;

    // dispense ball for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltDispenseCommand(beltSubsystem),
        new WaitCommand(0.5)
      )
    );

    // exit the inner terminal area going strait
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(2)));

    // intake ball for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltIntakeCommand(beltSubsystem),
        new WaitCommand(0.7)
      )
    );

    // go to the hub (backwards)
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(1), true));

    // dispense ball for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltDispenseCommand(beltSubsystem),
        new SequentialCommandGroup(
          new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(2)),
          new WaitCommand(0.5)
        )
      )
    );

    // exit the inner terminal area going strait
    addCommands();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setPosition(points.getStartingPosition());

    super.initialize();
  }
}
