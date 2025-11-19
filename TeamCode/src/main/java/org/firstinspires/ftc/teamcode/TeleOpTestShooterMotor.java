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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
@TeleOp(name="TestShooterMotor", group="Robot")
//@Disabled
public class TeleOpTestShooterMotor extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorEx  shootMotor   = null;


    public static double maxShootMotorRpm = 6000;
    public static double targetSpeedRpm = 0;
    static final double TICKS_PER_REVOLUTION = 28;

    @Override
    public void runOpMode() {

        ElapsedTime timer = new ElapsedTime();
        int prevPosition = 0;

        // Define and Initialize Motors
        shootMotor  = hardwareMap.get(DcMotorEx.class, "shooter_motor");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        prevPosition = shootMotor.getCurrentPosition();
        timer.reset();



        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        targetSpeedRpm =  0.0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            targetSpeedRpm = targetSpeedRpm + -gamepad1.left_stick_y * maxShootMotorRpm / 100.0;


            // Normalize the values so that we are within max rpm
            if (targetSpeedRpm > maxShootMotorRpm) {
                targetSpeedRpm = maxShootMotorRpm;
            } else if (targetSpeedRpm < 0.0) {
                targetSpeedRpm = 0.0;
            }

            double targetTicksPerSecond = (targetSpeedRpm / 60) * TICKS_PER_REVOLUTION;
            // Output the safe vales to the motor drives.
            shootMotor.setVelocity(targetTicksPerSecond);
            int curPosition = shootMotor.getCurrentPosition();
            double elapsedTimeSeconds = timer.seconds();
            timer.reset();
            double velocityTicksPerSecond = shootMotor.getVelocity();
            double curVelocityRpm = velocityTicksPerSecond * 60 / TICKS_PER_REVOLUTION;

            double ourRpmCalc = 60.0 * (curPosition - prevPosition) / TICKS_PER_REVOLUTION / elapsedTimeSeconds;

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Speed (Rpm)", targetSpeedRpm );
            packet.put("Current Speed (Rpm)", curVelocityRpm);
            packet.put("Loop Period (s)", elapsedTimeSeconds);
            packet.put("Our Speed Calc (Rpm)", curVelocityRpm);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // sleep(50); // TODO perhaps deleteme
        }
    }
}
