package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Stilt Test (Auto Run)", group = "Test")
public class StiltTestTeleop extends OpMode {

    // Change these names to match your Robot Config
    private static final String LEFT_STILT_NAME  = "leftFront";
    private static final String RIGHT_STILT_NAME = "rightRear";

    private DcMotorEx leftStilt;
    private DcMotorEx rightStilt;

    @Override
    public void init() {
        leftStilt  = hardwareMap.get(DcMotorEx.class, LEFT_STILT_NAME);
        rightStilt = hardwareMap.get(DcMotorEx.class, RIGHT_STILT_NAME);

        leftStilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightStilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftStilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightStilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure one motor is reversed so both drive the stilts the same direction mechanically
        leftStilt.setDirection(DcMotor.Direction.REVERSE); // reverse exactly one

        telemetry.addLine("Stilt Test: ready (will auto-run at START)");
        telemetry.update();
    }

    @Override
    public void start() {
        // Auto-run as soon as OpMode starts
        leftStilt.setPower(0.3);
        rightStilt.setPower(0.3);
    }

    @Override
    public void loop() {
        // Hold power on every loop (in case SDK clips it)
        leftStilt.setPower(0.3);
        rightStilt.setPower(0.3);

        telemetry.addData("Left Stilt Power", leftStilt.getPower());
        telemetry.addData("Right Stilt Power", rightStilt.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        leftStilt.setPower(0.0);
        rightStilt.setPower(0.0);
    }
}
