// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberSubsystem.CLIMBER_STATE;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.*;
import frc.robot.sensors.DistanceSensor;
import frc.robot.sim.Simulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // used for selecting autonomous
  private StatusSwitch statusSwitch = new StatusSwitch();

  private Simulation sim = new Simulation();

  //private final Limelight limelight = new Limelight();

  // sensors
  private final DistanceSensor distanceSensor = new DistanceSensor();

  // subsystems
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(sim);
  private final BeltSubsystem beltSubsystem = new BeltSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(distanceSensor);
  private final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();

  // commands
  private final DriveCommand driveCommand = new DriveCommand(driveTrainSubsystem);
  private final TargetBallCommand targetBallCommand = new TargetBallCommand(driveTrainSubsystem);
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
    Controller.Manipulator.getClimberUpButton().whenPressed(new InstantCommand(() -> {climberSubsystem.setClimberState(CLIMBER_STATE.UP);}, climberSubsystem));

    Controller.Manipulator.getWinchDownButton().whileHeld(new InstantCommand(() -> {climberSubsystem.set(.7);}, climberSubsystem));
    Controller.Manipulator.getWinchUpButton().whileHeld(new InstantCommand(() -> {climberSubsystem.set(-.7);}, climberSubsystem));

    Controller.Manipulator.getTargetBallButton().whenPressed(targetBallCommand);
  }

  private void configureDefaultCommands(){
    compressorSubsystem.setDefaultCommand(compressorCommand);
    driveTrainSubsystem.setDefaultCommand(driveCommand);
    climberSubsystem.setDefaultCommand(new RunCommand(() -> {climberSubsystem.set(0);}, climberSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // only run Autonomous when the status switch is at one
    if (statusSwitch.GetSwitchValue() != 1)
    {
      return null;
    }

    // generate autonomous command
    return new SequentialCommandGroup(
      // go to the hub
      new FollowPathCommand(driveTrainSubsystem, Autonomous.AUTONOMOUS[0].getTragectory(0, false)),

      // dispense ball for 0.5 seconds
      new ParallelRaceGroup(
        new BeltDispenseCommand(beltSubsystem),
        new WaitCommand(0.5)
      ),

      // exit the inner terminal area
      new FollowPathCommand(driveTrainSubsystem, Autonomous.AUTONOMOUS[0].getTragectory(1, true))
    );
  }

  public void scheduleTeleOpCommands() {
    // commands that will run on respective subsystems if no other commands are running
    //driveTrain.setDefaultCommand(driveCommand);
  }
}
