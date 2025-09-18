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

package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="RoboAvengers Timer Auton", group="Robot")
//@Disabled
public class FTCRoboAvengersTimerAuton extends LinearOpMode
{
    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null; //the left back drivetrain motor
    public DcMotor  rightBackDrive   = null; //the right back drivetrain motor
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  liftMotor        = null;
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null; //the claw head servo//the slider

    // Declare contants
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 70 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 25 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET     = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_HIGH_BASKET2     = 100 * ARM_TICKS_PER_DEGREE;
    final double LIFT_TICKS_PER_MM = 537.7 / 120.0;
    final double LIFT_SCORING_IN_HIGH_BASKET = 475 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_SAMPLE = 145 * LIFT_TICKS_PER_MM;
    static final double     FORWARD_SPEED = 0.35;
    static final double     TURN_SPEED    = 0.5;
    static final double     STRAFE_SPEED  = 0.35;
    final double CLAW_OPEN   = 0.0;
    final double CLAW_CLOSED  = 1.0;


    /* Variables that are used to set the arm to a specific position */
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.dcMotor.get("frontLeftMotor");
        leftBackDrive   = hardwareMap.dcMotor.get("backLeftMotor");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRightMotor");
        rightBackDrive  = hardwareMap.dcMotor.get("backRightMotor");
        armMotor        = hardwareMap.dcMotor.get("left_arm");
        liftMotor       = hardwareMap.dcMotor.get("liftMotor");
        claw  = hardwareMap.get(Servo.class, "claw");
        clawHead = hardwareMap.get(Servo.class, "clawHead");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // Step 1. Claw closed
        claw.setPosition(CLAW_OPEN);
        while (opModeIsActive() && (runtime.milliseconds() < 250) ) // [TBT] Reduced from 1000 to 250
        {
            telemetry.addData("Step 1: Claw closed", claw.getPosition());
            telemetry.update();
        }
        sleep(100); //[TBT] Reduced from 250 to 100ms
        runtime.reset();

        // Step 2. Lift and extend the arm for scoring
        double armPosition = (int)ARM_SCORE_HIGH_BASKET;
        armMotor.setPower(0.3);
        double liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        liftMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.milliseconds() < 1500) ) // Do not change as we require time for arm to stabilize
        {
            claw.setPosition(CLAW_OPEN);
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 2: Robot arm ready for the top scoring basket: ", "Complete");
        telemetry.update();
        sleep(100); //[TBT] Reduced from 250 to 100
        runtime.reset();

        // Step 3:  Drive forward for towards basket
        leftFrontDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Fwd Drive 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 3: Path to basket: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        // Step 4 Sample drop in top basket
        claw.setPosition(CLAW_CLOSED); //[TBT] Moved outside the while loop
        while (opModeIsActive() && (runtime.milliseconds() < 500)) //[TBT] Reduced from 1000 to 500
        {
            telemetry.addData("Step 4: Sample dropped: ", "Complete");
            telemetry.update();
        }

        sleep(100); //[TBT] Reduced from 500 to 100
        runtime.reset();

        //Step 5 Reverse the Robot
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 1050))
        {
            telemetry.addData("Path", "Rev Drive 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Step 5: Reverse the robot: ", "Complete");
        telemetry.update();


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);

        sleep(250);
        runtime.reset();

        // Step 6. Retract the robot arm and position for sample pickup
        armPosition = (int)ARM_CLEAR_BARRIER;
        armMotor.setPower(0.3);
        liftPosition = 0.0;
        liftMotor.setPower(0.1);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        while (opModeIsActive() && (runtime.milliseconds() < 1000) ) //[TBT] Reduced from 1500 to 1000
        {
            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 6: Retract the robot arm and position for sample pickup: ", "Complete");
        telemetry.update();
        sleep(100); //[TBT] Reduced from 250 to 100
        runtime.reset();

        //Step 7 Strafe to right
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(STRAFE_SPEED);
        rightFrontDrive.setPower(STRAFE_SPEED);
        leftBackDrive.setPower(STRAFE_SPEED);
        rightBackDrive.setPower(STRAFE_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 2350))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Step 7: Strafe right: ", "Complete");
        telemetry.update();
        sleep(100);
        runtime.reset();

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0); //[TODO] We need to test without setting power to zero
        liftMotor.setPower(0);

        // Step 8. Claw rotate
        clawHead.setPosition(0.0);
        while (opModeIsActive() && (runtime.milliseconds() < 500) )
        {
            telemetry.addData("Step 8: Claw closed", claw.getPosition());
            telemetry.update();
        }
        sleep(250);
        runtime.reset();

        //Step 9: Position arm extension to collect second sample
        liftPosition = LIFT_SCORING_IN_SAMPLE;
        liftMotor.setPower(0.2);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive() && (runtime.milliseconds() < 500) ) //[TBT] Reduced from 1000 to 500
        {
            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 9: Position arm extension to collect second sample: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        // Step 10. Lift the arm for scoring second sample
        armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
        armMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.milliseconds() < 1000) )
        {
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 10: Lift the arm for scoring second sample: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        // Step 10. Claw closed
        claw.setPosition(CLAW_OPEN);
        while (opModeIsActive() && (runtime.milliseconds() < 250) ) //[TBT] Reduced from 1000 to 250
        {
            telemetry.addData("Step 10: Claw closed", claw.getPosition());    //
            telemetry.update();
        }
        sleep(100 );//[TBT] Reduced from 250 to 100
        runtime.reset();

        // Step 11. Lift the arm for scoring
        armPosition = (int)ARM_SCORE_HIGH_BASKET;
        armMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.milliseconds() < 500) ) //[TBT] Reduced from 1000 to 500
        {
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 11: Lift the arm for scoring: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        //Step 12 Strafe to left
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(STRAFE_SPEED);
        rightFrontDrive.setPower(STRAFE_SPEED);
        leftBackDrive.setPower(STRAFE_SPEED);
        rightBackDrive.setPower(STRAFE_SPEED);

        while (opModeIsActive() && (runtime.milliseconds() < 2350))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Step 12: Strafe left: ", "Complete");
        telemetry.update();
        sleep(100);
        runtime.reset();

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);

        // Step 13. Extend the arm for second scoring
        armPosition = (int)ARM_SCORE_HIGH_BASKET2;
        armMotor.setPower(0.3);
        liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
        liftMotor.setPower(0.3);
        while (opModeIsActive() && (runtime.milliseconds() < 1000) ) //Reduced from 1500 to 1000
        {
            claw.setPosition(CLAW_OPEN);
            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 13: Robot arm angle ready for the scoring basket: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        armMotor.setPower(0);
        liftMotor.setPower(0);

        //Step 14: Set claw to middle scoring position
        clawHead.setPosition(0.65);
        while (opModeIsActive() && (runtime.milliseconds() < 250) ) //[TBT] Reduced from 500 to 250
        {
            telemetry.addData("Step 12: Set claw to middle scoring position", claw.getPosition());
            telemetry.update();
        }
        sleep(100); //[TBT] Reduced from 250 to 100
        runtime.reset();

        // Step 15:  Drive forward for 1.5 seconds
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Fwd Drive 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Step 15: Path to basket: ", "Complete");
        telemetry.update();
        sleep(250);
        runtime.reset();

        // Step 16 Basket drop
        claw.setPosition(CLAW_CLOSED);
        while (opModeIsActive() && (runtime.milliseconds() < 250)) //[TBT] Reduced from 1000 to 250
        {
            telemetry.addData("Claw closed", claw.getPosition());
            telemetry.update();
        }

        telemetry.addData("Step 16: Second sample dropped: ", "Complete");
        telemetry.update();
        sleep(100); //[TBT] Reduced from 250 to 100
        runtime.reset();

        // Reverse
        //Step 17 Reverse the robot for teleop
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);

        while (opModeIsActive() && (runtime.milliseconds() < 1000))
        {
            telemetry.addData("Path", "Leg 5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        telemetry.addData("Step 17: Reverse the robot for teleop: ", "Complete");
        telemetry.update();


        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        armMotor.setPower(0);
        liftMotor.setPower(0);

        sleep(100); //[TBT] Reduced from 250 to 100
        runtime.reset();

        // Step 18. Bring arm and lift to zero position
        armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
        armMotor.setPower(0.3);
        liftPosition = 0.0;
        liftMotor.setPower(0.1);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        while (opModeIsActive() && (runtime.milliseconds() < 1000) )
        {
            liftMotor.setTargetPosition((int) (liftPosition));
            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setTargetPosition((int) (armPosition));
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("Step 18: Robot arm closed: ", "Complete");
        telemetry.update();
        sleep(100);//[TBT] Reduced from 250 to 100
        runtime.reset();
        armMotor.setPower(0); //[TBT]
        liftMotor.setPower(0); //[TBT]
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}