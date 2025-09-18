/*
 * RoboAvengers Copyright 2024
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 *
 * There are four main components, mecanum drive, a arm for vertical reach, a linear slide for reaching
 * into the submersible, and a rotatable claw
 *
 * The Mecanum drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 *
 * The arm shoulder uses the same 117rpm motor with an external 5:1 gear reduction
 *
 * The Slider system uses a 5203-2402-0019 (312 RPM Yellow Jacket Motors)
 * The drivetrain is set up as "Robot centric" with the internal control hub IMU.
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
 */

@TeleOp(name="RoboAvengers TeleOp", group="Robot")
//@Disabled
public class FTCOnBotRoboAvengersTeleOp extends LinearOpMode
{
    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null; //the left front drivetrain motor
    public DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    public DcMotor  leftBackDrive    = null; //the left back drivetrain motor
    public DcMotor  rightBackDrive   = null; //the right back drivetrain motor
    public DcMotor  armMotor         = null; //the arm motor
    public DcMotor  liftMotor        = null; //the slider
    public Servo    claw             = null; //the claw servo
    public Servo    clawHead         = null; //the claw head servo

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    //[RA] Changed the angle from 76 to 70
    final double ARM_SCORE_SPECIMEN        = 70 * ARM_TICKS_PER_DEGREE;
    //[RA] Changed the angle from 90 to 80
    final double ARM_SCORE_SAMPLE_IN_LOW   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 140 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the positions that the claw should be set to when folding in, or folding out. */
    // Pranav made the CLAW_OPEN from 0.32 to 0.75 for custom claw
    final double CLAW_OPEN   = 0.0;
    final double CLAW_CLOSED  = 1.0;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    // [RA} Change for motor with 312RPM
    //final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;
    final double LIFT_TICKS_PER_MM = 537.7 / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 470 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;

    @Override
    public void runOpMode()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;


        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("frontLeftMotor");
        leftBackDrive   = hardwareMap.dcMotor.get("backLeftMotor");
        rightFrontDrive = hardwareMap.dcMotor.get("frontRightMotor");
        rightBackDrive  = hardwareMap.dcMotor.get("backRightMotor");
        liftMotor       = hardwareMap.dcMotor.get("liftMotor");
        armMotor        = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor

       /* [RA] TODO - Review this setting. We need to reverse the left side of the drivetrain
          so it doesn't turn when we ask all the drive motors to go forward */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        claw  = hardwareMap.get(Servo.class, "claw");
        clawHead = hardwareMap.get(Servo.class, "clawHead");

        /* Make sure that the intake is off, and the claw is folded in. */
        claw.setPosition(CLAW_OPEN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robo Avengers Steelbreaker is Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));


            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and claw) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the armPosition to the ARM_COLLECT position,
            it folds out the claw to make sure it is in the correct orientation to intake, and it
            turns the intake on to the COLLECT mode.*/

            if(gamepad1.a)
            {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                claw.setPosition(CLAW_OPEN);
            }
            else if (gamepad1.b)
            {
                /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the claw position or the intake power when we
                    select this "mode", this means that the intake and claw will continue what
                    they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.x)
            {
                /* This is the correct height to score the sample in the HIGH BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            else if (gamepad1.dpad_left)
            {
                /* This turns off the intake, folds in the claw, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                liftPosition = LIFT_COLLAPSED;
                claw.setPosition(CLAW_OPEN);
            }
            else if (gamepad1.dpad_right)
            {
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                //claw.setPosition(CLAW_CLOSED); // commented for game
            }
            else if (gamepad1.dpad_up)
            {
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                claw.setPosition(CLAW_CLOSED);
            }


            /*
            This is probably my favorite piece of code on this robot. It's a clever little software
            solution to a problem the robot has.
            This robot has an extending lift on the end of an arm shoulder. That arm shoulder should
            run to a specific angle, and stop there to collect from the field. And the angle that
            the shoulder should stop at changes based on how long the arm is (how far the lift is extended)
            so here, we add a compensation factor based on how far the lift is extended.
            That comp factor is multiplied by the number of mm the lift is extended, which
            results in the number of degrees we need to fudge our arm up by to keep the end of the arm
            the same distance from the field.
            Now we don't need this to happen when the arm is up and in scoring position. So if the arm
            is above 45°, then we just set armLiftComp to 0. It's only if it's below 45° that we set it
            to a value.
             */

            /*if (armPosition < 45 * ARM_TICKS_PER_DEGREE)
            {
                armLiftComp = (0.25568 * liftPosition);
            }
            else
            {
                armLiftComp = 0;
            }*/

           /* Here we set the target position of our arm to match the variable that was selected
            by the driver. We add the armPosition Variable to our armPositionFudgeFactor, before adding
            our armLiftComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/

            //armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
            armLiftComp = 0;
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Here we set the lift position based on the driver input.
            This is a.... weird, way to set the position of a "closed loop" device. The lift is run
            with encoders. So it knows exactly where it is, and there's a limit to how far in and
            out it should run. Normally with mechanisms like this we just tell it to run to an exact
            position. This works a lot
like our arm. Where we click a button and it goes to a position, then stops.
            But the drivers wanted more "open loop" controls. So we want the lift to keep extending for
            as long as we hold the bumpers, and when we let go of the bumper, stop where it is at.
            This allows the driver to manually set the position, and not have to have a bunch of different
            options for how far out it goes. But it also lets us enforce the end stops for the slide
            in software. So that the motor can't run past it's endstops and stall.
            We have our liftPosition variable, which we increment or decrement for every cycle (every
            time our main robot code runs) that we're holding the button. Now since every cycle can take
            a different amount of time to complete, and we want the lift to move at a constant speed,
            we measure how long each cycle takes with the cycletime variable. Then multiply the
            speed we want the lift to run at (in mm/sec) by the cycletime variable. There's no way
            that our lift can move 2800mm in one cycle, but since each cycle is only a fraction of a second,
            we are only incrementing it a small amount each cycle.
             */

            if (gamepad2.right_bumper)
            {
                liftPosition += 2800 * cycletime;
            }
            else if (gamepad2.left_bumper)
            {
                liftPosition -= 2800 * cycletime;
            }
            else if (gamepad2.x)
            {
                claw.setPosition(CLAW_CLOSED);
            }
            else if (gamepad2.y)
            {
                claw.setPosition(CLAW_OPEN);
            }
            else if (gamepad2.dpad_down)
            {
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                claw.setPosition(CLAW_OPEN);
            }
            else if (gamepad2.a)
            {
                /* THIS IS FOR COLLECTING SAMPLES */
                //armPosition = ARM_WINCH_ROBOT;
                clawHead.setPosition(0.0);
            }
            else if (gamepad2.b)
            {
                /* THIS IS FOR COLLECTING SAMPLES */
                //armPosition = ARM_WINCH_ROBOT;
                clawHead.setPosition(1.0);
            }
            else if (gamepad2.dpad_left)
            {
                clawHead.setPosition(0.25);
            }
            else if (gamepad2.dpad_right)
            {
                clawHead.setPosition(0.6);
            }
            else if (gamepad2.dpad_up)
            {
                clawHead.setPosition(0.4);
            }
            /*here we check to see if the lift is trying to go higher than the maximum extension.
             *if it is, we set the variable to the max.
             */
            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET)
            {
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }
            //same as above, we see if the lift is trying to go below 0, and if it is, we set it to 0.
            if (liftPosition < 0)
            {
                liftPosition = 0;
            }

            liftMotor.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) liftMotor).setVelocity(2100);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent())
            {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* This is how we check our loop time. We create three variables:
            looptime is the current time when we hit this part of the code
            cycletime is the amount of time in seconds our current loop took
            oldtime is the time in seconds that the previous loop started at

            we find cycletime by just subtracting the old time from the current time.
            For example, lets say it is 12:01.1, and then a loop goes by and it's 12:01.2.
            We can take the current time (12:01.2) and subtract the oldtime (12:01.1) and we're left
            with just the difference, 0.1 seconds.*/
            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("Left Front Motor Position: ", leftFrontDrive.getCurrentPosition());
            telemetry.addData("Right Front Motor Position: ", rightFrontDrive.getCurrentPosition());
            telemetry.addData("Left Back Motor Position: ", leftBackDrive.getCurrentPosition());
            telemetry.addData("Right Back Motor Position: ", rightBackDrive.getCurrentPosition());

            telemetry.addData("Left Front Motor Current:",((DcMotorEx) leftFrontDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Front Motor Current:",((DcMotorEx) rightFrontDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left Back Motor Current:",((DcMotorEx) leftBackDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Right Back Motor Current:",((DcMotorEx) rightBackDrive).getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Arm Target Position: ", armMotor.getTargetPosition());
            telemetry.addData("Arm Current Position: ", armMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Current:",((DcMotorEx) armMotor).getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Slide Motor Current:",((DcMotorEx) liftMotor).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Slider Target Position",liftMotor.getTargetPosition());
            telemetry.addData("Slider Current position", liftMotor.getCurrentPosition());

            telemetry.addData("Claw Servo position:",claw.getPosition());
            telemetry.addData("Claw Head Servo position:",clawHead.getPosition());

            telemetry.update();
        }
    }
}
