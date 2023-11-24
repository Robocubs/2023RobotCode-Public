package com.team1701.lib.field;

import java.util.HashMap;

public class AprilTagSection {
    public final int kAprilTagId;
    public CubeColumn mCubeColumn;
    public ConeColumn mLeftConeColumn;
    public ConeColumn mRightConeColumn;
    // L, M, R
    public HashMap<Integer, Column> mColumns;

    // An AprilTagSection is just a triplet of columns whose lineup points are all determined
    // via an AprilTag on the central cube column
    public AprilTagSection(int id, int leftMostColumnID) {
        super();
        kAprilTagId = id;

        mColumns = new HashMap<Integer, Column>(3);
        mCubeColumn = new CubeColumn(id);
        mLeftConeColumn = new ConeColumn(id, true);
        mRightConeColumn = new ConeColumn(id, false);
        mColumns.put(leftMostColumnID, mLeftConeColumn);
        mColumns.put(leftMostColumnID + 1, mCubeColumn);
        mColumns.put(leftMostColumnID + 2, mRightConeColumn);
    }
}
