package org.firstinspires.ftc.teamcode.turret;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    private final YawServo yawServo;
    private final LaunchMotor launchMotor;

    public Turret(boolean front, HardwareMap hardwareMap, Follower follower, String yawServoName, String launchMotorName){
        this.yawServo   = new YawServo(hardwareMap, follower, yawServoName, front);
        this.launchMotor = new LaunchMotor(hardwareMap, follower, launchMotorName);
    }

    /** Call this every loop */
    public void updateTurret(){
        // 1) Spin up/adjust flywheel speed for the current pose/physics
        launchMotor.updateLaunchMotor();

        // 2) Point turret toward the goal (or your intercept if you compute it elsewhere)
        // If you later expose (x_int, y_int) from TurretConstants, plug them in here instead.
        yawServo.updateDirection(TurretConstants.x0, TurretConstants.y0);
    }
}
