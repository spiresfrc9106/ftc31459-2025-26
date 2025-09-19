package org.firstinspires.ftc.teamcode.turret;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.util.Range;
public class YawServo {

    private final Servo servo;
    private final Follower follower;

    // --- Calibration (set these on your bot) ---
    // Mechanical mapping: yaw -maxDeg .. +maxDeg -> servo pos min..max
    private double SERVO_MIN_POS = 0.12;   // measured
    private double SERVO_MAX_POS = 0.88;   // measured
    private double YAW_MIN_DEG   = -120.0; // left limit
    private double YAW_MAX_DEG   =  120.0; // right limit

    // Mechanical zero offset (deg). If turret faces field +X when servo is midway, set this to 0.
    // If your “forward” points a few degrees off, set that bias here.
    private double YAW_OFFSET_DEG = 0.0;

    // Motion smoothing
    private static final double MAX_DEG_STEP = 8.0; // per loop

    // Direction flip if your linkage is mirrored
    // State
    private double lastCmdDeg = 0.0;


    /**
     * Initializes new servo for the turret pivot
     * @param hardwareMap hardware map of robot
     * @param follower pedro follower for current x and y of robot
     * @param name name of servo on hardware map
     * @param reversed Whether or not servo is reversed, depends on which side of the bot it is on
     */
    public YawServo(HardwareMap hardwareMap, Follower follower, String name, boolean reversed) {
        this.follower = follower;
        this.servo = hardwareMap.get(Servo.class, name);
        this.servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Optional: update calibration at runtime if you measure new endpoints. */
    public void calibrate(double yawMinDeg, double yawMaxDeg, double servoMinPos, double servoMaxPos, double yawOffsetDeg) {
        this.YAW_MIN_DEG   = yawMinDeg;
        this.YAW_MAX_DEG   = yawMaxDeg;
        this.SERVO_MIN_POS = servoMinPos;
        this.SERVO_MAX_POS = servoMaxPos;
        this.YAW_OFFSET_DEG = yawOffsetDeg;
    }

    public void updateDirection(double targetX, double targetY){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();                     // <-- fixed bug (was getX)
        double robotHeadingRad = follower.getPose().getHeading();      // Pedro heading (radians)

        // Bearing from robot to target (field frame)
        double bearingRad = Math.atan2(targetY - robotY, targetX - robotX);

        // Yaw error in robot frame: how much to rotate base
        double errRad = normalizeRad(bearingRad - robotHeadingRad);
        double errDeg = Math.toDegrees(errRad) + YAW_OFFSET_DEG;

        // Apply soft limits and slew limit
        double clamped = Range.clip(errDeg, YAW_MIN_DEG, YAW_MAX_DEG);
        double step = Range.clip(clamped - lastCmdDeg, -MAX_DEG_STEP, MAX_DEG_STEP);
        lastCmdDeg += step;

        // Map degrees -> servo position
        double t = (lastCmdDeg - YAW_MIN_DEG) / (YAW_MAX_DEG - YAW_MIN_DEG);
        double pos = SERVO_MIN_POS + t * (SERVO_MAX_POS - SERVO_MIN_POS);
        servo.setPosition(Range.clip(pos, 0.0, 1.0));

    }

    public double getCommandedYawDeg() { return lastCmdDeg; }
    // ----- helpers -----

    /**
     *
     * @param a angle in radians
     * @return normalized radians such that the angle is between -pi to pi
     */
    private static double normalizeRad(double a) {
        while (a > Math.PI)  a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }


}
