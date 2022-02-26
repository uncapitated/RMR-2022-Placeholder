// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

/** Add your docs here. */
public class StatusSwitch {
    @Getter
    public DigitalInput inputs[] = new DigitalInput[4];

    public StatusSwitch()
    {
        inputs[0] = new DigitalInput(Constants.StatusSwitch.FIRST_INPUT);
        inputs[1] = new DigitalInput(Constants.StatusSwitch.SECOND_INPUT);
        inputs[2] = new DigitalInput(Constants.StatusSwitch.THIRD_INPUT);
        inputs[3] = new DigitalInput(Constants.StatusSwitch.FOURTH_INPUT);
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


