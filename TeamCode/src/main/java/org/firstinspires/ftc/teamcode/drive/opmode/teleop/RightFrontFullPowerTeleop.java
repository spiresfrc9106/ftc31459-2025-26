package org.firstinspires.ftc.teamcode.drive.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RightFront Full Power", group = "Test")
public class RightFrontFullPowerTeleop extends OpMode {

    private DcMotor rightFront;

    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("RightFront ready");
        telemetry.update();
    }

    @Override
    public void start() {
        rightFront.setPower(1.0); // full forward
    }

    @Override
    public void loop() {
        // Keep it pinned at full power
        rightFront.setPower(1.0);
        telemetry.addData("rightFront power", rightFront.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        rightFront.setPower(0.0);
    }
}
