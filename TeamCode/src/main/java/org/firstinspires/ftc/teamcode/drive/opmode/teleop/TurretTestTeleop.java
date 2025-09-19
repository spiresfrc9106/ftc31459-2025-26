package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.turret.LaunchMotor;
import org.firstinspires.ftc.teamcode.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.turret.YawServo;

@TeleOp(name = "Turret Test Teleop", group = "Test")
public class TurretTestTeleop extends OpMode {

    // Drive / localization
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    // Turret
    private YawServo yawServo;
    private LaunchMotor launchMotor;

    // Simple state
    private boolean launcherEnabled = true; // toggle with A
    private boolean lastAToggle = false;
    private boolean frontMount = true; // set to false if your yaw servo is mounted reversed

    @Override
    public void init() {
        // Pedro follower init
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        // Turret devices
        // Replace names with your actual config names
        yawServo = new YawServo(hardwareMap, follower, "yawServo", frontMount);
        launchMotor = new LaunchMotor(hardwareMap, follower, "launcher");

        telemetry.addLine("Turret Test Teleop: INIT complete");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                false
        );
        follower.update();

        // --- Toggle launcher enable with A (edge-detected) ---
        boolean aNow = gamepad1.a;
        if (aNow && !lastAToggle) {
            launcherEnabled = !launcherEnabled;
        }
        lastAToggle = aNow;

        // --- Aim yaw at the field goal each loop ---
        yawServo.updateDirection(TurretConstants.x0, TurretConstants.y0);

        // --- Spin/adjust flywheel to target RPM (based on pose + physics) ---
        if (launcherEnabled) {
            launchMotor.updateLaunchMotor();   // computes target RPM and commands motor velocity
        } else {
            // If you prefer to hard-stop the motor when disabled, add a stop() method in LaunchMotor
            // that does launchMotor.setVelocity(0). For now we simply skip updates.
        }

        // --- Telemetry ---
        telemetry.addLine("== Drive/Pose ==");
        telemetry.addData("X (in)", follower.getPose().getX());
        telemetry.addData("Y (in)", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("== Turret ==");
        telemetry.addData("Launcher Enabled", launcherEnabled ? "YES" : "NO");
        // Optional: show computed target RPM without commanding (calculateMotorSpeed() is read-safe)
        double previewRpm = launchMotor.calculateMotorSpeed();
        telemetry.addData("Target Flywheel RPM", String.format("%.1f", previewRpm));
        telemetry.addData("Aim Target", "(x0,y0)=(" + TurretConstants.x0 + ", " + TurretConstants.y0 + ")");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Optional: add a stop() helper in LaunchMotor that calls setVelocity(0)
        // and set the yaw servo to a safe neutral.
    }
}
