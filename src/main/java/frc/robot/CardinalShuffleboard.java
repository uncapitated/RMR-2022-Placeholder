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

import frc.robot.subsystems.DriveTrain;

/**
 * Interface for populating Shuffleboard with the Robot state All new widgets
 * will be found in the Cardinal Tab in Shuffleboard
 */
public class CardinalShuffleboard {

    private static NetworkTableEntry maxForwardPowerEntry;
    private static NetworkTableEntry maxTurnPowerEntry;
    private static NetworkTableEntry currentProtectionEnabledEntry;

    private static ShuffleboardTab cardinalTab = Shuffleboard.getTab("Cardinal");

    // block for active commands
    private static ShuffleboardLayout commandsLayout = cardinalTab.getLayout("Commands", BuiltInLayouts.kList)
            .withSize(2, 3).withPosition(0, 0);

    // block for Errors
    private static ShuffleboardLayout errorsLayout = cardinalTab.getLayout("Errors", BuiltInLayouts.kList)
            .withSize(2, 1).withPosition(0, 3);

    // block for Driver
    private static ShuffleboardLayout driveTrainLayout = cardinalTab.getLayout("Drive Train", BuiltInLayouts.kList)
            .withSize(3, 3).withPosition(6, 0);

    // Main block
    private static ShuffleboardLayout mainLayout = cardinalTab.getLayout("Main", BuiltInLayouts.kList)
            .withSize(4, 3).withPosition(2, 0);

    private static ShuffleboardLayout fieldLayout = cardinalTab.getLayout("Field", BuiltInLayouts.kList)
            .withSize(7, 3).withPosition(2, 3);

    public static void setupMainLayout(DifferentialDrive drive, PowerDistribution powerDistributionPanel) {
        mainLayout.add(drive).withWidget(BuiltInWidgets.kDifferentialDrive);
        mainLayout.add(powerDistributionPanel).withWidget(BuiltInWidgets.kPowerDistribution);
    }

    public static void setupErrorsLayout() {
        errorsLayout.add("Current Limit Exceeded", false).withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("color when true", "FF0000", "color when false", "000000"));
    }

    public static void setError(String name, boolean value) {
        errorsLayout.add(name, value);
    }

    public static void setupFieldLayout(){
    }

    public static void setupDriveTrainLayout(DriveTrain driveTrain, double maxForwardPower, double maxTurnPower) {
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

    public static void setCurrentProtectionCommand(CommandBase currentMonitorCommand) {
        currentProtectionEnabledEntry.addListener(event -> {
            if (event.value.getBoolean()) { // if the switch is turned on then the current monitor command is disabled
                currentMonitorCommand.schedule();
            }
            else {
                currentMonitorCommand.cancel();
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
