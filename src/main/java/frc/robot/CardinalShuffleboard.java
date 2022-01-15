// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LaunchWheels;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Levitation;

/**
 * Interface for populating Shuffleboard with the Robot state All new widgets
 * will be found in the Cardinal Tab in Shuffleboard
 */
public class CardinalShuffleboard {
    private static ShuffleboardTab cardinalTab = Shuffleboard.getTab("Cardinal");

    // block for active commands
    private static ShuffleboardLayout commandsLayout = cardinalTab.getLayout("Commands", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(0, 0);

    // block for Errors
    private static ShuffleboardLayout errorsLayout = cardinalTab.getLayout("Errors", BuiltInLayouts.kList)
            .withSize(2, 1).withPosition(0, 3);

    // block for Driver
    private static ShuffleboardLayout driveTrainLayout = cardinalTab.getLayout("Drive Train", BuiltInLayouts.kList)
            .withSize(2, 4).withPosition(6, 0);
    private static NetworkTableEntry maxForwardPowerEntry;
    private static NetworkTableEntry maxTurnPowerEntry;

    //block for Arm Wheels
    private static ShuffleboardLayout wheelsLayout = cardinalTab.getLayout("Arm Wheels", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(0, 0);
    private static NetworkTableEntry currentPowerEntry;

    // block for elevator
    private static ShuffleboardLayout elevatorShuffleboardLayout = cardinalTab.getLayout("Elevator", BuiltInLayouts.kList)
            .withSize(1,4).withPosition(8, 0);
    private static NetworkTableEntry maxRotationSpeed;

    // Main block
    private static ShuffleboardLayout mainLayout = cardinalTab.getLayout("Main", BuiltInLayouts.kList)
            .withSize(4, 4).withPosition(2, 0);

    public static void setupMainLayout(DifferentialDrive drive) {
        mainLayout.add(drive).withWidget(BuiltInWidgets.kDifferentialDrive);
    }

    public static void setupErrorsLayout() {
        errorsLayout.add("Current Limit Exceded", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "FF0000", "color when false", "000000"));
    }

    public static void setError(String name, boolean value) {
        errorsLayout.add(name, value);
    }

    public static void setupArmWheelsLayout(LaunchWheels launchWheels, boolean aPress) {
        wheelsLayout.add(launchWheels);
        currentPowerEntry = wheelsLayout.addPersistent("Current Power", aPress).withWidget(BuiltInWidgets.kBooleanBox)
            .getEntry();
    }

    public static void setupDriveTrainLayout(DriveTrain driveTrain, double maxForwardPower, double maxTurnPower) {
        maxForwardPowerEntry = driveTrainLayout.addPersistent("Max Forward Power", maxForwardPower)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

        maxTurnPowerEntry = driveTrainLayout.addPersistent("Max Turn Power", maxTurnPower)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();


    }

    public static double getMaxForwardPowerEntry() {
        return maxForwardPowerEntry.getDouble(1.0);
    }

    public static double getMaxTurnPowerEntry() {
        return maxTurnPowerEntry.getDouble(1.0);
    }

 

    public static void setupCommandsLayout(CommandBase... commands)
    {
        for (CommandBase command : commands) {
            commandsLayout.add(command).withWidget(BuiltInWidgets.kCommand);
        }
    }

    public static void setupElevatorLayout(Levitation levitations)
    {
        maxRotationSpeed = elevatorShuffleboardLayout.addPersistent("Current Elevator Speed", levitations.get_levitation())
                .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1)).getEntry();
    }
}
