package com.team1701.lib.arm;

import com.team1701.lib.util.GeometryUtil;
import com.team1701.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;

public class ArmSetpoint {
    public final Rotation2d angle;
    public final double extension;

    public ArmSetpoint() {
        angle = GeometryUtil.kRotationIdentity;
        extension = 0;
    }

    public ArmSetpoint(Rotation2d angle, double extension) {
        this.angle = angle;
        this.extension = extension;
    }

    @Override
    public String toString() {
        return "ArmSetpoint(Angle: " + angle.getDegrees() + " deg, Extension: " + extension + ")";
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof ArmSetpoint)) {
            return false;
        }

        var other = (ArmSetpoint) obj;
        return angle.equals(other.angle) && Util.epsilonEquals(extension, other.extension);
    }
}
