package frc.robot.sensors;

import java.util.Objects;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class DistanceSensor {
    private Rev2mDistanceSensor sensor = null;

    public DistanceSensor(){
        try {
            sensor = new Rev2mDistanceSensor(Port.kOnboard);
        } catch (Exception e){
            System.out.println("Distance Sensor not initialized");
        }
        sensor.setAutomaticMode(true);
    }

    public double getDistance(){
        if (Objects.nonNull(sensor)){
            return sensor.getRange();
        } else {
            return -1;
        }
    }
}
