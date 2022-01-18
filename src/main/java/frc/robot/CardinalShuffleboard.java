// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LaunchWheels;
import frc.robot.subsystems.DriveTrain;

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
            .withSize(2, 3).withPosition(0, 3);

    // block for Driver
    private static ShuffleboardLayout driveTrainLayout = cardinalTab.getLayout("Drive Train", BuiltInLayouts.kList)
            .withSize(3, 6).withPosition(8, 0);
    
    private static NetworkTableEntry maxForwardPowerEntry;
    private static NetworkTableEntry maxTurnPowerEntry;
    private static NetworkTableEntry currentProtectionEnabledEntry;

    //block for Arm Wheels
    private static ShuffleboardLayout wheelsLayout = cardinalTab.getLayout("Arm Wheels", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(0, 0);
    private static NetworkTableEntry currentPowerEntry;

    // Main block
    private static ShuffleboardLayout mainLayout = cardinalTab.getLayout("Main", BuiltInLayouts.kList)
            .withSize(6, 6).withPosition(2, 0);

    public static void setupMainLayout(DifferentialDrive drive, PowerDistribution panel) {
        mainLayout.add(drive).withWidget(BuiltInWidgets.kDifferentialDrive);
        mainLayout.add(panel).withWidget(BuiltInWidgets.kPowerDistribution);
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
        driveTrainLayout.add(driveTrain);
        maxForwardPowerEntry = driveTrainLayout.addPersistent("Max Forward Power", maxForwardPower)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

        maxTurnPowerEntry = driveTrainLayout.addPersistent("Max Turn Power", maxTurnPower)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();

        currentProtectionEnabledEntry = driveTrainLayout.add("Enable Current Protection", false)
                .withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    }

    public static double getMaxForwardPowerEntry() {
        return maxForwardPowerEntry.getDouble(1.0);
    }

    public static double getMaxTurnPowerEntry() {
        return maxTurnPowerEntry.getDouble(1.0);
    }

    public static void setCurrentProtectionCommand(CommandBase currentMoniterCommand) {
        currentProtectionEnabledEntry.addListener(event -> {
            if (event.value.getBoolean()) { // if the switch is turned on then the current moniter command is disabled
                currentMoniterCommand.schedule();
            }
            else {
                currentMoniterCommand.cancel();
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public static void setupCommandsLayout(CommandBase... commands)
    {
        for (CommandBase command : commands) {
            commandsLayout.add(command).withWidget(BuiltInWidgets.kCommand);
        }
    }
}
