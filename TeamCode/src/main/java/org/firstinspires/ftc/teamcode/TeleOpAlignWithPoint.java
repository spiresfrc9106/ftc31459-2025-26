package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// TODO import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.TankDrive;
// TODO import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * This opmode demonstrates how one would implement "align to point behavior" in teleop. You specify
 * a desired vector (x/y coordinate) via`targetPosition`. In the `ALIGN_TO_POINT` mode, the bot will
 * switch into field centric control and independently control its heading to align itself with the
 * specified `targetPosition`.
 * <p>
 * Press `a` to switch into alignment mode and `b` to switch back into standard teleop driving mode.
 * <p>
 * Note: We don't call drive.update() here because it has its own field drawing functions. We don't
 * want that to interfere with our graph so we just directly update localizer instead
 */
@Config
@TeleOp(group = "advanced")
public class TeleOpAlignWithPoint extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDController headingController = new PIDController(0,0,0, 0
            // TODO SampleMecanumDrive.HEADING_PID
    );

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(new Vector2d(-36.0,0), Math.toRadians(180));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        //drive.localizer.setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        // TODO headingController.setInputBounds(-Math.PI, Math.PI);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.localizer.getPose();

            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            PoseVelocity2d driveDirection = new PoseVelocity2d( new Vector2d(0,0),0);

            telemetry.addData("mode", currentMode);

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (gamepad1.a) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new PoseVelocity2d( new Vector2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x),-gamepad1.right_stick_x);
                    break;
                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.b) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInputRotatedMinus90 = new Vector2d(
                            -gamepad1.left_stick_x,
                            -1 * -gamepad1.left_stick_y
                    );
                    Vector2d robotFrameInput = poseEstimate.heading.vec().times(-1); //  = -poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.heading.vec()); //  = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angleCast().toDouble();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = fieldFrameInputRotatedMinus90.dot(difference) / (difference.norm() * difference.norm());
                    // = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    // TODO not the right class for heading control headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = 0.0;/* TODO (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH; */

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new PoseVelocity2d(
                            robotFrameInput,
                            headingInput
                    );

                    /* TODO
                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());

                    */
                    break;
            }

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            //TODO DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

            drive.setDrivePowers(driveDirection);

            // Update the heading controller with our current heading
            // TODO headingController.update(poseEstimate.getHeading());

            // Update he localizer
            drive.localizer.update();

            // Send telemetry packet off to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.heading.component1());
            telemetry.addData("y", poseEstimate.heading.component1());
            telemetry.addData("heading", poseEstimate.heading);
            telemetry.update();
        }
    }
}
