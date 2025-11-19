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
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode executes control of the viper motor from teleop
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick forward back changes the target speed.
 *
 */
@Config
@TeleOp(name="TestViperMotor", group="Robot")
//@Disabled
public class TeleOpTestViperMotor extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx viperMotor = null;


    public static double maxShootMotorRpm = 300;
    public static double targetSpeedRpm = 0;
    static final double TICKS_PER_REVOLUTION = 1425.1;

    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();
        int prevPosition = 0;

        // Define and Initialize Motors
        viperMotor = hardwareMap.get(DcMotorEx.class, "viper");
        viperMotor.setDirection(DcMotorEx.Direction.REVERSE);
        viperMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        prevPosition = viperMotor.getCurrentPosition();
        timer.reset();

        // Wait for the game to start (driver presses START)
        waitForStart();

        targetSpeedRpm =  0.0;

        double oldPositionTicks = viperMotor.getCurrentPosition();
        double prevLeftStickWithDeadZone = 100;
        double holdPositionTicks = 0.0;
        boolean hold = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double elapsedTimeSeconds = timer.seconds();
            double targetPeriodSeconds = 0.020;


            if (elapsedTimeSeconds < targetPeriodSeconds) {
                sleep(1);
            } else {

                double leftStickWithDeadZone = -gamepad1.left_stick_y;
                if (Math.abs(leftStickWithDeadZone) < 0.05) {
                    leftStickWithDeadZone = 0;
                }

                if (!hold && (leftStickWithDeadZone == 0) && prevLeftStickWithDeadZone != 0) {
                    hold = true;
                    holdPositionTicks = viperMotor.getCurrentPosition();
                    oldPositionTicks = holdPositionTicks;
                } else if (leftStickWithDeadZone !=0) {
                    hold = false;
                }

                int curPositionTicks = viperMotor.getCurrentPosition();

                double targetTicksPerSecond = 0;

                if (hold) {
                    targetTicksPerSecond = -0.2*(curPositionTicks - holdPositionTicks) / targetPeriodSeconds;
                } else {
                    targetSpeedRpm = leftStickWithDeadZone * maxShootMotorRpm;

                    targetTicksPerSecond = (targetSpeedRpm / 60) * TICKS_PER_REVOLUTION;
                }

                // The viper slide is only driven up, it coasts down.
                if (targetTicksPerSecond<0) {
                    targetTicksPerSecond = 0;
                }

                viperMotor.setVelocity(targetTicksPerSecond);

                double velocityTicksPerSecond = viperMotor.getVelocity();
                double curVelocityRpm = velocityTicksPerSecond * 60 / TICKS_PER_REVOLUTION;

                double ourRpmCalc = 60.0 * (curPositionTicks - prevPosition) / TICKS_PER_REVOLUTION / elapsedTimeSeconds;

                int holdInt = 0;
                if (hold)
                    holdInt = 1;

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Target Speed (Rpm)", targetSpeedRpm);
                packet.put("Current Speed (Rpm)", curVelocityRpm);
                packet.put("Current Pos (ticks)", curPositionTicks);
                packet.put("hold Pos (ticks)", holdPositionTicks);
                packet.put("Loop Period (s)", elapsedTimeSeconds);
                packet.put("Our Speed Calc (Rpm)", curVelocityRpm);
                packet.put("Busy (fraction)", (timer.seconds() - elapsedTimeSeconds) / targetPeriodSeconds);
                packet.put("holdInt", holdInt);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                timer.reset();

            }
        }

    }
}
