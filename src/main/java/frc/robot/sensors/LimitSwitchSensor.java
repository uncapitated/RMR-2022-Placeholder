// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import java.util.Objects;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import lombok.Getter;

/** Add your docs here. */
public class LimitSwitchSensor {
    private DigitalInput limitSwitch;

    @Getter
    private DIOSim limitSwitchSim;

    public LimitSwitchSensor(int dioPort) {
        try {
            limitSwitch = new DigitalInput(dioPort);
        } catch (Exception e) {
            limitSwitch = null;
            System.out.println("Limit Switch with DIO port " + dioPort + " not initialized");
        }
        if (RobotBase.isSimulation()){
            limitSwitchSim = new DIOSim(limitSwitch);
            limitSwitchSim.setValue(true); // off
        }
    }

    public boolean isPressed() {
        if (Objects.nonNull(limitSwitch)){
            return !limitSwitch.get();
        } else {
            //if it isn't connected, it should just be like it isn't pressed
            return true;
        }
    }
}
