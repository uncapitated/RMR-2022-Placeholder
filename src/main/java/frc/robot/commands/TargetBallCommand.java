// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CameraPIDConstants;
import frc.robot.Controller;
import frc.robot.Constants.CameraConstants;
import frc.robot.Controller.Drive;
import frc.robot.subsystems.DriveTrainSubsystem;


import org.json.*;

public class TargetBallCommand extends CommandBase {

    NetworkTable table;
    NetworkTableEntry coral, detections;

    int cameraXSize = 160;
    int cameraYSize = 120;

    private DriveTrainSubsystem driveTrain;

    private PIDController tpid;

    double xmin, ymin, xmax, ymax, avX, degrees, confidence;

    private double turnSpeed;

    private JSONObject jsObj;
    private JSONArray jsAr,jsAr2;

    int area, currMax, currMaxPos;
    /** Creates a new TargetBallCommand. */
    public TargetBallCommand(DriveTrainSubsystem driveTrain) {

      this.driveTrain = driveTrain;

      tpid = new PIDController(CameraPIDConstants.akP, CameraPIDConstants.akI, CameraPIDConstants.akD);

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      table = NetworkTableInstance.getDefault().getTable("ML");
      coral = table.getEntry("coral");
      detections = table.getEntry("detections");
      
      // Get the resolution of the camera from Network Tables
      // todo
      // NetworkTableEntry resolutionNetworkTableEntry = table.getEntry("resolution");
      // String resolutionIntegers = resolutionNetworkTableEntry.getValue().toString();
      // if (resolutionInteger.length == 0)
      // cameraXSize = Integer.parseInt(resolutionIntegers.substring(0, resolutionIntegers.indexOf(',')));
      // cameraYSize = Integer.parseInt(resolutionIntegers.substring(resolutionIntegers.indexOf(", ")));
      
      

      //Note: this is for use to check if the coral is attached, first must check how this is sent to NetworkTables. 
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
      //if(table.getEntry("label").getString("").equals(CameraConstants.label[0])) {
      //jsObj = new JSONObject(table.getEntry("box").getString(""));
        jsAr = new JSONArray(detections.getString("[]"));

        currMax = 0;
        
        for(int i = 0; i < jsAr.length(); i++)
        {
          jsAr2 = new JSONArray(jsAr.getJSONArray(i));
          jsObj = jsAr2.getJSONObject(1);
          xmin = jsObj.getDouble("xmin");
          ymin = jsObj.getDouble("ymin");
          xmax = jsObj.getDouble("xmax");
          ymax = jsObj.getDouble("ymax");

          area = (int)((xmax-xmin)*(ymax-ymin));

          if(area > currMax){
            currMax = area;
            currMaxPos = i;
          }
        }

        jsObj = jsAr.getJSONObject(currMaxPos);
        xmin = jsObj.getDouble("xmin");
        ymin = jsObj.getDouble("ymin");
        xmax = jsObj.getDouble("xmax");
        ymax = jsObj.getDouble("ymax");
        confidence = table.getEntry("confidence").getDouble(0);

        avX = (xmin+xmax)/2;

        degrees = (cameraXSize/2-avX)/(cameraXSize/2)*180;

        turnSpeed = -1*tpid.calculate(degrees, 0);

        System.out.println(xmin + " " + xmax + " " + degrees + " " + confidence);
        if(Math.abs(degrees) >= 5) {
          driveTrain.set(new ChassisSpeeds(0, 0, turnSpeed));
        } else {
          driveTrain.set(new ChassisSpeeds(.3,0,0));
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
