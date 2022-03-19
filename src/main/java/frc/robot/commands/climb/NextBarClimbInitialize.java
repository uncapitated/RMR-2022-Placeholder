// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NextBarClimbInitialize extends SequentialCommandGroup {
  /** Creates a new NextBarClimbInitialize. */
  public NextBarClimbInitialize(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /**
     * requires:
     * robot to be hanging from a bar on the lower hooks
     * 
     * steps:
     * 1) go to transition point
     * 2) set to angled
     * 3) fully extend
     * 4) set to out (to put pressure on the bar)
     */
    
    // move to transition point to change climber angle
    addCommands(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {return climberSubsystem.atSetPoint();}), // makes sure that the elevator is at the right position before ending the command
        new RunCommand(()->{climberSubsystem.setPositionTransition();}, climberSubsystem)
      )
    );

    // move the climber in (angled)
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(()->{climberSubsystem.setClimberIn();}, climberSubsystem),
        new WaitCommand(0.5) // wait 0.5 seconds to move the climber to the up position
      )
    );

    // move to top point to grab bar
    addCommands(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {return climberSubsystem.atSetPoint();}), // makes sure that the elevator is at the right position before ending the command
        new RunCommand(()->{climberSubsystem.setPositionTop();}, climberSubsystem)
      )
    );

    // move the climber out (not angled) to put pressure on the bar
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(()->{climberSubsystem.setClimberOut();}, climberSubsystem),
        new WaitCommand(0.5) // wait 0.5 seconds to move the climber to the up position
      )
    );
  }
}
