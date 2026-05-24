// REBELLION 10014

package frc.lib.math;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /* Really small epsilon, just a reminder */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, 1e-9);
    }

    public static boolean inBetween(double value, double min, double max) {
        return value > min && value < max;
    }

    public static double getClosest(double value, double value1, double value2) {
        if(Math.abs(value1 - value) > Math.abs(value2 - value)) {
            return value2;
        } else return value1;
    }

    /** Extension methods for wpi geometry objects */
    public static class GeomExtensions {
        public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
            return EqualsUtil.epsilonEquals(twist.dx, other.dx)
                    && EqualsUtil.epsilonEquals(twist.dy, other.dy)
                    && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
        }
    }
}
