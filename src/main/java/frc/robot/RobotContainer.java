// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.ResourceBundle.Control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberSubsystem.CLIMBER_STATE;
import frc.robot.subsystems.DriveTrainSubsystem.SHIFTER_POSITION;
import frc.robot.Constants.Autonomous;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.ComplexAutoCommand;
import frc.robot.commands.autonomous.SimpleAutoCommand;
import frc.robot.sensors.ClimberSensorCollection;
import frc.robot.sensors.DistanceSensor;
import frc.robot.sensors.LimitSwitchSensor;
import frc.robot.simulation.Simulation;

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

  // sensors (doesn't have sim support)
  private final ClimberSensorCollection climberSensors = new ClimberSensorCollection();

  // subsystems
  private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(sim);
  private final BeltSubsystem beltSubsystem = new BeltSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem(climberSensors);
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
    Controller.Drive.getIntakeButton().whenHeld(intakeCommand);
    Controller.Drive.getDispenseButton().whenHeld(dispenseCommand);

    // This is disabled because we use this solenoid for venting rather than shifting gears
    // Controller.Drive.getSlowButton().whenPressed(new InstantCommand(() -> driveTrainSubsystem.setShifter(SHIFTER_POSITION.LOW)));
    // Controller.Drive.getSlowButton().whenReleased(new InstantCommand(() -> driveTrainSubsystem.setShifter(SHIFTER_POSITION.HIGH)));
    driveTrainSubsystem.setShifter(SHIFTER_POSITION.LOW);

    Controller.Manipulator.getVentButton().whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> driveTrainSubsystem.setShifter(SHIFTER_POSITION.LOW)),
      new WaitCommand(0.2),
      new InstantCommand(() -> driveTrainSubsystem.setShifter(SHIFTER_POSITION.HIGH))
    ));
 
  
    // less elegant but it actually works
    // old one folded down whenever you moved the elevator, which we don't want
    Controller.Manipulator.getToggleButton().whenPressed(new InstantCommand(() -> {
      if (climberSubsystem.getClimberState() == CLIMBER_STATE.IN){
        climberSubsystem.setClimberState(CLIMBER_STATE.OUT);
      } else {
        climberSubsystem.setClimberState(CLIMBER_STATE.IN);
      }
    }, climberSubsystem));

    // the safety button
    Controller.Manipulator.getSafetyButton().whileHeld(new InstantCommand(() -> climberSubsystem.incrementSafetyCounter()));

    Controller.Manipulator.getElevatorDownButton().whileActiveContinuous(new ParallelCommandGroup(new InstantCommand(() -> {climberSubsystem.set(Controller.Manipulator.getSlowButton().get() ? 0.25 : 0.5);}, climberSubsystem), new CoastCommand(driveTrainSubsystem)));
    Controller.Manipulator.getElevatorUpButton().whileActiveContinuous(new ParallelCommandGroup(new InstantCommand(() -> {climberSubsystem.set(Controller.Manipulator.getSlowButton().get() ? -0.25 : -0.5);}, climberSubsystem), new CoastCommand(driveTrainSubsystem)));

    if (RobotBase.isReal()){
      // Controller.Drive.getAlignButton().whileHeld(targetBallCommand);
    }
  }

  private void configureDefaultCommands(){
    compressorSubsystem.setDefaultCommand(compressorCommand);
    driveTrainSubsystem.setDefaultCommand(driveCommand);
    climberSubsystem.setDefaultCommand(new RunCommand(() -> {climberSubsystem.set(0);}, climberSubsystem));
    beltSubsystem.setDefaultCommand(new RunCommand(() -> {beltSubsystem.stop();}, beltSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println(statusSwitch.GetSwitchValue());

    // use simulated status switch to control witch autonomous to use
    switch(statusSwitch.GetSwitchValue())
    {
      case 1:
      return new SimpleAutoCommand(driveTrainSubsystem, beltSubsystem, Autonomous.AUTONOMOUS[0]);

      case 2:
      return new ComplexAutoCommand(driveTrainSubsystem, beltSubsystem, Autonomous.AUTONOMOUS[1]);

      default:
      return null;
    }
  }

  /**
   * setup auto variables
   */
  public void startAutonomous()
  {
    //driveTrainSubsystem.setPosition(Autonomous.AUTONOMOUS[0].getStartingPosition());
  }

  public void scheduleTeleOpCommands() {
    // commands that will run on respective subsystems if no other commands are running
    //driveTrain.setDefaultCommand(driveCommand);
  }
}
