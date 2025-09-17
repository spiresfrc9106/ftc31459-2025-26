package org.firstinspires.ftc.teamcode.turret;

public class TurretConstants {
    public static final double G = 386.4; // gravity acceleration converted from feet into inches, 32.2*12 = 386.4
    public static final double PHI = Math.toRadians(45); // turret angle
    public static final double x0 = 144; // goal location (inches)
    public static final double y0 = 144; // goal location (inches)`
    public static final double h = 38.75; // goal location height (inches)
    public static final double h_clear = 0.25;
    public static final double sz = 15; // height of turret in inches
    public double sx, sy; // robot pos

    public double v_ix, v_iy; // robot velocity, idk how this gunna be calculated
    public double x_int, y_int; // target position, needs to be defined based on localization coordinate system
    public static final double h_target = h + h_clear; // target position height for power calc
    public static final double h_delta = h_target - sz; // height difference between the turret and the hoop

    public double theta; // angle of turret relative to goal
    public double distance; // distance to goal


    public double velocity; // velocity for launching the balls, used to calculate motor rpm
    public double vx, vy, vz; // velocity components from shooter

    public double v_mx, v_my; // effective velocity relative to field

    public double theta_m, v_m; // corrected aim angle and horizontal magnitude

    // wrote a quick method that recalculates all the turret values each time it's called, assuming sx, sy, and sz are defined
    public void update_turret_values() {
        x_int = ((11.0/12)*(x0+y0)+(y0-sy)/(x0-sx)*sx -sy)/(1+(y0-sy)/(x0-sx)); // target x location
        y_int = (11.0/12)*(x0+y0)-x_int; // target y location
        // azimuth angle and distance
        theta = Math.toDegrees(Math.atan2(y_int - sy, x_int - sx));
        distance = Math.sqrt(Math.pow(x_int - sx, 2) + Math.pow(y_int - sy, 2));

        velocity = distance * Math.sqrt(G / (distance - h_delta)); // velocity

        // velocity components
        vx = velocity * Math.cos(PHI) * Math.cos(theta);
        vy = velocity * Math.cos(PHI) * Math.sin(theta);
        vz = velocity * Math.sin(PHI);

        // Effective velocity after subtracting robot motion
        v_mx = vx - v_ix;
        v_my = vy - v_iy;

        // Corrected aim angle and effective horizontal speed, IMPORTANT THINGY HERE
        theta_m = Math.toDegrees(Math.atan2(v_my, v_mx));
        v_m = Math.sqrt(v_mx * v_mx + v_my * v_my);
    }

    public static final double  WHEEL_DIAMETER = 1.88976; // diameter of the flywheel

    // run after update_turret_values to ensure v_m is calculated
    public double getTargetRPM() {
        return (v_m * 60.0) / (Math.PI * WHEEL_DIAMETER);
    }
}