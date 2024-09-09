package org.firstinspires.ftc.teamcode.Libs.Classes;

public class CUnits {
    /**
     * @param sq - squares
     * @return - ticks
     */
    public int sq_to_tk(double sq) {
        double tk = sq * 1100;
        return (int) tk;
    }

    /**
     * @param sq - squares
     * @return - ticks
     */
    public int sq_to_tk_S(double sq) {
        double tk = sq * 1200;
        return (int) tk;
    }

    /**
     * @param deg - degrees
     * @return - squares
     */

    public int deg_to_tk(double deg, double power) {
        double rv = (-200*(power)) + 890;
        double tk = (deg / 90) * rv;
        return (int) tk;
    }
}