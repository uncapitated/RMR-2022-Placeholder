package frc.robot.sensors;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensor {
    private Rev2mDistanceSensor sensor = new Rev2mDistanceSensor(Port.kOnboard);

    public DistanceSensor(){
        sensor.setAutomaticMode(true);
    }

    public double getDistance(){
        return sensor.getRange();
    }
}
