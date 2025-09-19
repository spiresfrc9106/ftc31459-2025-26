package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Computes the flywheel RPM from field-relative physics in {@link TurretConstants}
 * and (optionally) the robot’s current linear velocity estimated from the Follower pose.
 *
 * Call {@link #calculateMotorSpeed()} each loop to get the target RPM.
 * You can then feed that RPM to your velocity controller (e.g., PIDF on motor).
 */
public class LaunchMotor {
    private final DcMotorEx launchMotor;
    private final Follower follower;
    private final TurretConstants tc;

    // simple finite-difference state to estimate robot velocity (in/s) if the follower doesn’t expose it
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private long lastTimeNanos = 0L;

    public LaunchMotor(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.launchMotor = hardwareMap.get(DcMotorEx.class, motorName);
        this.follower = follower;
        this.tc = new TurretConstants();

        // Configure motor for velocity control upstream if you plan to use setVelocity
        // launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void updateLaunchMotor(){
        // 1) Compute target flywheel RPM from pose/physics
        double targetFlywheelRpm = calculateMotorSpeed();

        // 2) Convert to motor ticks/second (assume 1:1 motor→flywheel unless you call setMotorVelocityFromRpm with a ratio)
        double ticksPerRev = launchMotor.getMotorType().getTicksPerRev();
        if (Double.isNaN(targetFlywheelRpm) || Double.isInfinite(targetFlywheelRpm)) return;
        if (targetFlywheelRpm < 0) targetFlywheelRpm = 0;

        double motorRpm = targetFlywheelRpm; // gearRatioMotorToFlywheel = 1.0 by default
        double ticksPerSecond = motorRpm * ticksPerRev / 60.0;

        // 3) Command the motor
        launchMotor.setVelocity(ticksPerSecond);
    }



    /**
     * Returns target flywheel speed in RPM based on current pose and estimated robot XY velocity.
     * You can also set {@link TurretConstants#sx}, {@link TurretConstants#sy},
     * {@link TurretConstants#v_ix}, and {@link TurretConstants#v_iy} directly
     * before calling this if you prefer to supply your own localization/velocity.
     */
    public double calculateMotorSpeed() {
        // --- Pose (inches) ---
        // Pedro Pathing uses inches for typical FTC fields; adjust if your config differs.
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        // --- Estimate robot linear velocity v_ix, v_iy (in/s) via finite difference ---
        long now = System.nanoTime();
        double vIx = 0.0, vIy = 0.0;

        if (!Double.isNaN(lastX) && now > lastTimeNanos) {
            double dt = (now - lastTimeNanos) * 1e-9; // seconds
            if (dt > 1e-3) { // avoid division blow-ups on the first couple of loops
                vIx = (x - lastX) / dt;
                vIy = (y - lastY) / dt;
            }
        }

        lastX = x;
        lastY = y;
        lastTimeNanos = now;

        // --- Wire into turret physics model ---
        tc.sx = x;
        tc.sy = y;
        tc.v_ix = vIx;
        tc.v_iy = vIy;

        // Recompute intercept, aim, and effective muzzle speed
        tc.update_turret_values();

        // Convert to flywheel RPM
        return tc.getTargetRPM();
    }

    /**
     * Optional helper: set motor velocity using encoder ticks/second.
     * Provide your motor’s ticksPerRev and a gearRatio from motor shaft to flywheel (motor revs / flywheel rev).
     */
    public void setMotorVelocityFromRpm(double targetFlywheelRpm, double ticksPerRev, double gearRatioMotorToFlywheel) {
        // motor RPM = flywheel RPM * gearRatio
        double motorRpm = targetFlywheelRpm * gearRatioMotorToFlywheel;
        double ticksPerSecond = motorRpm * ticksPerRev / 60.0;
        launchMotor.setVelocity(ticksPerSecond);
    }


}
