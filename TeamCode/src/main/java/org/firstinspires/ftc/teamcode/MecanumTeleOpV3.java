/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode executes control of the shooter motro from teleop
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick forward back changes the target speed.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="MecanumTeleOpV3", group="Robot")
//@Disabled
public class MecanumTeleOpV3 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx  shootMotor   = null;


    public static double maxShootMotorRpm = 6000;
    public static double targetSpeedRpm = 0;
    static final double TICKS_PER_REVOLUTION = 28;


    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        // Tom and Sammy: You really wanted the y 24 to be -24
        Pose2d initialPose = new Pose2d(new Vector2d(-68,-24), Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ElapsedTime timer1 = new ElapsedTime();

        // Wait for the game to start (driver presses START)
        waitForStart();

        List<Action> doActions = new ArrayList<>();

        boolean start = false;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();

            double xySpeedFactor = 0.25;
            double rotationSpeedFactor = 0.25;

            double x_vel_frac = -gamepad1.left_stick_y; // Sammy note the minus sign here
            double y_vel_frac = -gamepad1.left_stick_x;
            double rot_vel_frac = -gamepad1.right_stick_x;

            if (Math.abs(x_vel_frac)<0.05) {
                x_vel_frac = 0;
            }

            if (Math.abs(y_vel_frac)<0.05) {
                y_vel_frac = 0;
            }

            if (Math.abs(rot_vel_frac)<0.05) {
                rot_vel_frac = 0;
            }

            double xIPS = x_vel_frac * drive.PARAMS.maxWheelVel * xySpeedFactor;
            double yIPS = y_vel_frac * drive.PARAMS.maxWheelVel * xySpeedFactor;
            double rotRadPS = rot_vel_frac * drive.PARAMS.maxAngVel * rotationSpeedFactor;


            packet.put("x IPS", xIPS);
            packet.put("y IPS", xIPS);

            packet.put("rot DPS", Math.toDegrees(rotRadPS));

            drive.setFieldRelativeDrive(xIPS, yIPS, rotRadPS);

            // Update everything. Odometry. Etc.
            drive.localizer.update();

            Canvas c = drive.sendPlotData(packet);


            for (Action action : doActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            doActions = newActions;

            packet.put("Runtime (s)", getRuntime());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
