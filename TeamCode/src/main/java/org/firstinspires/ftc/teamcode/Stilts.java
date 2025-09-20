package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Stilts {
    private final DcMotorEx stiltLeft;
    private final DcMotorEx stiltRight;

    // Tune these as needed
    private static final double DEFAULT_EXTEND_POWER  = 1.0;   // push stilts down to lift robot
    private static final double DEFAULT_RETRACT_POWER = -0.8;  // pull stilts up
    private boolean enabled = false; // for simple toggle use in TeleOp

    public Stilts(HardwareMap hardwareMap, String stiltLeftName, String stiltRightName){
        stiltLeft  = hardwareMap.get(DcMotorEx.class, stiltLeftName);
        stiltRight = hardwareMap.get(DcMotorEx.class, stiltRightName);

        // Recommended setup
        stiltLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stiltRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stiltLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stiltRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If your motors are mirrored mechanically, you may need to reverse one in the RC config
        // or uncomment exactly one of these:
        // stiltLeft.setDirection(DcMotor.Direction.REVERSE);
        // stiltRight.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    /** Extend stilts (down) to lift robot. */
    public void extend() {
        setPowerBoth(DEFAULT_EXTEND_POWER);
        enabled = true;
    }

    /** Retract stilts (up). */
    public void retract() {
        setPowerBoth(DEFAULT_RETRACT_POWER);
        enabled = true;
    }

    /** Stop both stilts and hold with brake. */
    public void stop() {
        setPowerBoth(0.0);
        enabled = false;
    }

    /** Direct power control (same power to both). Range [-1, 1]. */
    public void setPowerBoth(double power){
        double p = Math.max(-1.0, Math.min(1.0, power));
        stiltLeft.setPower(p);
        stiltRight.setPower(p);
    }

    /** Simple toggle helper for a single button in TeleOp.
     *  Call this on button PRESS (edge-detected). When enabled, extends; press again to stop. */
    public void toggleExtend() {
        if (enabled) stop();
        else extend();
    }

    /** Optional: toggle retract on a different button. */
    public void toggleRetract() {
        if (enabled) stop();
        else retract();
    }
}
