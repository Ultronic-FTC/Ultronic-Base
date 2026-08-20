package org.firstinspires.ftc.teamcode.lib.util;

public class MathUtils {
    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }

    private static final double TAU = Math.PI * 2;


    /**
     * Returns {@code angle} clamped to [0, 2pi].
     *
     * @param angle angle measure in radians
     * @return normalized angle in range [0, 2*pi)
     */
    public static double norm(double angle) {
        double modified = angle % TAU;
        modified = (modified + TAU) % TAU;
        return modified;
    }

    /**
     * Returns {@code angleDelta} clamped to [-pi, pi].
     *
     * @param angleDelta angle delta in radians
     * @return normalized delta in range (-pi, pi]
     */
    public static double normDelta(double angleDelta) {
        double modified = norm(angleDelta);
        if (modified > Math.PI) {
            modified -= TAU;
        }
        return modified;
    }
}