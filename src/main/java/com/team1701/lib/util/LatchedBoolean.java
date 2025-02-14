package com.team1701.lib.util;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
    private boolean mLast = false;

    public boolean update(boolean newValue) {
        var ret = false;
        if (newValue && !mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}
