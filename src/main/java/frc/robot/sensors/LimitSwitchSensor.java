// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import lombok.Getter;

/** Add your docs here. */
public class LimitSwitchSensor {
    private DigitalInput limitSwitch;

    @Getter
    private DIOSim limitSwitchSim;

    public LimitSwitchSensor(int dioPort) {
        limitSwitch = new DigitalInput(dioPort);

        limitSwitchSim = new DIOSim(limitSwitch);
        limitSwitchSim.setValue(true); // off
    }

    public boolean isPressed() {
        return !limitSwitch.get();
    }
}
