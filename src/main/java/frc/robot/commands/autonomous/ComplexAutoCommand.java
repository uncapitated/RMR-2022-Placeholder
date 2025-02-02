// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutonomousPoints;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.BeltDispenseCommand;
import frc.robot.commands.BeltIntakeCommand;
import frc.robot.commands.DriveToPositionCommand;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexAutoCommand extends SequentialCommandGroup {
  /** Creates a new ComplexAutoCommand. */
  DriveTrainSubsystem driveTrain;
  AutonomousPoints points; 
  
  public ComplexAutoCommand(DriveTrainSubsystem driveTrainSubsystem, BeltSubsystem beltSubsystem, AutonomousPoints autoPoints) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    driveTrain = driveTrainSubsystem;
    points = autoPoints;

    //Go back to pick up second ball
    addCommands(
      new ParallelRaceGroup(
        new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(1)),
        new BeltIntakeCommand(beltSubsystem)
      )
    );

    // pick up balls for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltIntakeCommand(beltSubsystem),
        new WaitCommand(1)
      )
    );

    // intake ball for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltIntakeCommand(beltSubsystem),
        new WaitCommand(0.5)
      )
    );

    //Drive back to hub
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(2), true));

    // dispense balls for 0.5 seconds
    addCommands(
      new ParallelRaceGroup(
        new BeltDispenseCommand(beltSubsystem),
        new WaitCommand(1)
      )
    );

    //Drive back out of line
    addCommands(new DriveToPositionCommand(driveTrainSubsystem, autoPoints.getPosition(3)));


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setPosition(points.getStartingPosition());

    super.initialize();
  }
}
