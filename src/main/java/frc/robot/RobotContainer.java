// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberSubsystem.CLIMBER_STATE;
import frc.robot.commands.*;
import frc.robot.sim.Simulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //private Autonomous auto = new Autonomous();

  private Simulation sim = new Simulation();

  //private final Limelight limelight = new Limelight();

  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(sim);
  private final BeltSubsystem beltSubsystem = new BeltSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();

  // commands
  private final DriveCommand driveCommand = new DriveCommand(driveTrainSubsystem);
  
  private final BeltIntakeCommand intakeCommand = new BeltIntakeCommand(beltSubsystem);
  private final BeltDispenseCommand dispenseCommand = new BeltDispenseCommand(beltSubsystem);

  private final CompressorCommand compressorCommand = new CompressorCommand(compressorSubsystem);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Controller.Manipulator.getIntakeButton().whenHeld(intakeCommand);
    Controller.Manipulator.getDispenseButton().whenHeld(dispenseCommand);

    Controller.Manipulator.getClimberAngleButton().whenPressed(new InstantCommand(() -> {climberSubsystem.setClimberState(CLIMBER_STATE.ANGLED);}, climberSubsystem));
    Controller.Manipulator.getClimberAngleButton().whenPressed(new InstantCommand(() -> {climberSubsystem.setClimberState(CLIMBER_STATE.UP);}, climberSubsystem));

    Controller.Manipulator.getWinchInButton().whileHeld(new RunCommand(() -> {climberSubsystem.set(1);}, climberSubsystem));
    Controller.Manipulator.getWinchOutButton().whileHeld(new RunCommand(() -> {climberSubsystem.set(-1);}, climberSubsystem));
  }

  private void configureDefaultCommands(){
    compressorSubsystem.setDefaultCommand(compressorCommand);
    driveTrainSubsystem.setDefaultCommand(driveCommand);
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
    //driveTrain.setDefaultCommand(driveCommand);
  }
}
