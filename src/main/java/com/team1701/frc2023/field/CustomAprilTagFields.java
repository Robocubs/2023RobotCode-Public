package com.team1701.frc2023.field;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public enum CustomAprilTagFields {
    kBlueFieldUDJ("blueFieldUDJ.json"),
    kRedFieldUDJ("redFieldUDJ.json");

    public final String m_resourceFile;

    CustomAprilTagFields(String resourceFile) {
        m_resourceFile = new File(Filesystem.getDeployDirectory(), "fields/" + resourceFile).getAbsolutePath();
    }
}
