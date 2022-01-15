// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Levitation levitation = new Levitation();
  private final LaunchWheels launchWheels = new LaunchWheels();
  private final DriveCommand driveCommand = new DriveCommand(driveTrain);
  private final WingardiumLeviosa wingardiumLeviosa = new WingardiumLeviosa(levitation);
  private final WheelsCommand wheelsCommand = new WheelsCommand(launchWheels);
  private final SpinnyThing spinnyThing = new SpinnyThing();
  private final SpinTheSpinner spinTheSpinner = new SpinTheSpinner(spinnyThing);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    CardinalShuffleboard.setupDriveTrainLayout(driveTrain, driveCommand.getMaxForward(), driveCommand.getMaxTurn());
    CardinalShuffleboard.setupElevatorLayout(levitation);
    CardinalShuffleboard.setupMainLayout(driveTrain.getDrive());
    CardinalShuffleboard.setupErrorsLayout();
    // CardinalShuffleboard.setupArmWheelsLayout(launchWheels, Controller.Drive.get_a_button());

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void scheduleTeleOpCommands() {
    // command that will run on drive train when no other commands are running
    driveTrain.setDefaultCommand(driveCommand);
    levitation.setDefaultCommand(wingardiumLeviosa);
    launchWheels.setDefaultCommand(wheelsCommand);
    spinnyThing.setDefaultCommand(spinTheSpinner);

    // command responsible for checking PDP

  }

  public void checkForCommandsToSchedule()
  {
  }
}
