// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import lombok.Getter;
import lombok.Setter;

/** 
 * this is the interface to use with the threshholding vision system
 */
public class VisionInterface implements Updatable {
    /**
     * represents a threshold for a given object
     */
    public class Threshold {
        public String name;
        public int[] minThreshold;
        public int[] maxThreshold;

        public Threshold(String name, int[] min, int[] max) {
            this.name = name;
            minThreshold = min;
            maxThreshold = max;
        }

        public Threshold(String name) {
            this(name, new int[]{0, 0, 0},  new int[]{0, 0, 0});
        }
    }

    private final String[] entries = {"redBall", "blueBall"};

    private NetworkTable visionTable;

    // entries to communicate with the pi
    private NetworkTableEntry thresholdsEntry;
    private NetworkTableEntry filteringEntry;
    private NetworkTableEntry circleFilteringEntry;
    private NetworkTableEntry fillingEntry;

    private NetworkTableEntry detectionsEntry;
    private NetworkTableEntry resolutionEntry;

    // entries to communicate with human
    private ShuffleboardTab interfaceTab;
    private SendableChooser<Threshold> thresholdChooser;
    private Threshold lastThreshold;

    NetworkTableEntry selectedEntry;
    NetworkTableEntry[] minHSV;
    NetworkTableEntry[] maxHSV;

    private Map<String, Threshold> thresholds;
    
    public VisionInterface() {
        UpdateManager.getInstance().register(this);

        // setup communication with the robot
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        thresholdsEntry = visionTable.getEntry("thresholds");
        filteringEntry = visionTable.getEntry("filtering");
        circleFilteringEntry = visionTable.getEntry("circle filtering");
        fillingEntry = visionTable.getEntry("filling");

        detectionsEntry = visionTable.getEntry("detections");
        resolutionEntry = visionTable.getEntry("resolution");

        // read values
        thresholds = readThresholdSettings();

        // check if need to add new thresholds
        if (thresholds.size() < entries.length) {
            thresholds.clear();
            for (String entry : entries) {
                thresholds.put(entry, new Threshold("entry"));
            }

            writeThresholdSettings();
        }

        thresholdChooser.setDefaultOption(entries[0], thresholds.get(entries[0]));
        for (String entry : entries) {
            thresholdChooser.addOption(entry, thresholds.get(entry));
        }
        lastThreshold = thresholdChooser.getSelected();

        interfaceTab = Shuffleboard.getTab("VisionInterface");

        interfaceTab.add("Entry", thresholdChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 4);



        // setup widgets
        ShuffleboardLayout minHSVLayout = interfaceTab.getLayout("Min HSV").withPosition(2, 1).withSize(2, 3);
        minHSV[0] = minHSVLayout.add("H", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 180)).getEntry();
        minHSV[1] = minHSVLayout.add("S", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
        minHSV[2] = minHSVLayout.add("V", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();

        ShuffleboardLayout maxHSVLayout = interfaceTab.getLayout("Min HSV").withPosition(4, 1).withSize(2, 3);
        maxHSV[0] = maxHSVLayout.add("H", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 180)).getEntry();
        maxHSV[1] = maxHSVLayout.add("S", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();
        maxHSV[2] = maxHSVLayout.add("V", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 255)).getEntry();

        for (NetworkTableEntry individualThreshold : minHSV) {
            individualThreshold.addListener(entry -> {
            }, EntryListenerFlags.kUpdate | EntryListenerFlags.kNew);
        }
    }

    private Map<String, Threshold> readThresholdSettings() {
        JSONParser parser = new JSONParser();

        Map<String, Threshold> newThresholds;
        JSONArray newThresholdsJSON = new JSONArray();
        
        try {
            newThresholdsJSON = (JSONArray) parser.parse(thresholdsEntry.getString("[]"));
        } catch (ParseException e) {
            e.printStackTrace();
        }

        newThresholds = new HashMap<String, Threshold>();

        for (int i = 0; i < newThresholdsJSON.size(); i++) {
            JSONObject newThresholdJSON = (JSONObject) newThresholdsJSON.get(i);
            Threshold newThreshold = new Threshold((String)newThresholdJSON.get("name"),(int[])newThresholdJSON.get("bottom"),(int[])newThresholdJSON.get("top"));
            newThresholds.put(newThreshold.name, newThreshold);
        }

        return thresholds = newThresholds;
    }

    private String writeThresholdSettings() {
        JSONArray thresholdsJSON = new JSONArray();

        for (Threshold threshold : thresholds.values())
        {
            JSONObject thresholdJSON = new JSONObject();
            thresholdJSON.put("name", threshold.name);
            thresholdJSON.put("bottom", threshold.minThreshold);
            thresholdJSON.put("top", threshold.maxThreshold);

            thresholdsJSON.add(thresholdsJSON);
        }

        thresholdsEntry.setString(thresholdsJSON.toJSONString());

        return thresholdsJSON.toJSONString();
    }

    public void update() {
        // check for changes in the threshold selector
        if (thresholdChooser.getSelected() != lastThreshold) {
            lastThreshold = thresholdChooser.getSelected();
            for (int i = 0; i < 3; i++) {
                minHSV[0].setFlags(1);
            }
        }
    }
}
