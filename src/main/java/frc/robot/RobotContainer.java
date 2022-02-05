// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Autonomous auto = new Autonomous();

  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Limelight limelight = new Limelight();
  private final Escalator escalator = new Escalator();


  // commands
  private final DriveCommand driveCommand = new DriveCommand(driveTrain, limelight);
  private final LimelightAlignCommand limelightCommand = new LimelightAlignCommand(limelight, driveTrain);
  private final EscalatorCommand escalatorCommand = new EscalatorCommand(escalator);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    CardinalShuffleboard.setupDriveTrainLayout(driveTrain, driveCommand.getMaxForward(), driveCommand.getMaxTurn());
    CardinalShuffleboard.setupErrorsLayout();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Controller.Drive.getXAlignButton().whenPressed(limelightCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public void scheduleTeleOpCommands() {
    // commands that will run on respective subsystems if no other commands are running
    driveTrain.setDefaultCommand(driveCommand);
  }
}
