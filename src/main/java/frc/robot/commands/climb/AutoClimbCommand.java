// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * Pulls the robot up to the first Bar
 * 
 * WARNING the climber must be in the out position or bad things can happen
 */
public class AutoClimbCommand extends SequentialCommandGroup {
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    // go to the bottom position to hook onto the lower hooks
    addCommands(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {return climberSubsystem.atSetPoint();}), // makes sure that the elevator is at the right position before ending the command
        new RunCommand(()->{climberSubsystem.setPositionBottom();}, climberSubsystem)
      )
    );

    addCommands(new WaitCommand(1.0)); // wait 1.0 seconds

    // test the connection by raising the elevator slightly
    // go to the bottom position to hook onto the lower hooks
    addCommands(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(() -> {return climberSubsystem.atSetPoint();}), // makes sure that the elevator is at the right position before ending the command

        new RunCommand(()->{climberSubsystem.setPosition(Climber.MIN_OUT + 0.10);}, climberSubsystem) // moves to 10 cm above the lower hooks
      )
    );
  }
}
