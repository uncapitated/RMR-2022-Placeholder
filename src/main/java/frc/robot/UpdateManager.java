// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.LinkedList;
import java.util.List;

/** Add your docs here. */
public class UpdateManager {
    private static UpdateManager instance;

    private List<Updatable> items;

    public static UpdateManager getInstance() {
        if(instance == null)
        {
            instance = new UpdateManager();
            instance.items = new LinkedList<Updatable>();
        }
        return instance;
    }

    public void update() {
        for (Updatable item : items) {
            item.update();
        }
    }

    public void register(Updatable updatable) {
        if(!items.contains(updatable)) {
            items.add(updatable);
        }
    }
}
