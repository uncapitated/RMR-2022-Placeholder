// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import lombok.Getter;

/** Add your docs here. */
public class StatusSwitch {
    @Getter
    public DigitalInput inputs[] = new DigitalInput[4];

    public DIOSim inputsSim[] = new DIOSim[4];

    public ShuffleboardTab statusSwitchTab;

    public StatusSwitch()
    {
        inputs[0] = new DigitalInput(Constants.StatusSwitch.FIRST_INPUT);
        inputs[1] = new DigitalInput(Constants.StatusSwitch.SECOND_INPUT);
        inputs[2] = new DigitalInput(Constants.StatusSwitch.THIRD_INPUT);
        inputs[3] = new DigitalInput(Constants.StatusSwitch.FOURTH_INPUT);

        // setup shuffleboard
        statusSwitchTab = Shuffleboard.getTab("StatusSwitch");

        // setup simulation
        for(int i = 0; i < inputs.length; i++)
        {
            inputsSim[i] = new DIOSim(inputs[i]);

            inputsSim[i].setValue(true); // high is off

            DIOSim currentSim = inputsSim[i];

            NetworkTableEntry inputEntry = statusSwitchTab.add("DIO " + inputs[i].getChannel(), inputs[i].get()).withWidget(BuiltInWidgets.kToggleButton).getEntry();
            inputEntry.addListener(event -> {
                currentSim.setValue(event.value.getBoolean());
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        }
    }

    public int GetSwitchValue()
    {
        int value = 0;
        for (int i = 0; i < 4; i++)
        {
            if(!(inputs[i].get()))
                value += Math.pow(2,i);
        }
        return value;
    }
}


