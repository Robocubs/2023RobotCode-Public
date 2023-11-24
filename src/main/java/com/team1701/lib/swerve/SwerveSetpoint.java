// The core logic in this code was derived from Team 254's 2022 robot code
// https://github.com/Team254/FRC-2022-Public
// THE REPOSITORY RELEASED WITH THIS CODE WAS/IS UNDER THE MIT LICENSE
package com.team1701.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public ChassisSpeeds mChassisSpeeds;
    public SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    @Override
    public String toString() {
        String ret = mChassisSpeeds.toString() + "\n";
        for (int i = 0; i < mModuleStates.length; ++i) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}
