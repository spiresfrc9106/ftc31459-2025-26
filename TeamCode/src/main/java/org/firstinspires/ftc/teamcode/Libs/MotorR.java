package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libs.Classes.MotorCupling;

@TeleOp(name="MotorReset (DO NOT USE!!!)", group="Helper")
public class MotorR extends PlayOpMode{
    //Make Objects
    MecanumMove move;
    MotorCupling LBA; //Lower Base Arm
    DcMotor UPA; //Upper Power Arm

    //Controller
    double t = 0;
    double a1 = 0;
    double a2 = 0;
    //Vars
    double speed = 0.3;
    @Override
    protected void preInitialize() {
        isTeleOp = true;
        LBA = new MotorCupling();
    }

    @Override
    protected void initialize() {
        UPA = hardwareMap.get(DcMotor.class, "upa");

        LBA.DC1 = hardwareMap.get(DcMotor.class, "lba1");
        LBA.DC2 = hardwareMap.get(DcMotor.class, "lba2");

        //Set encoder
        UPA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("UPA", UPA.getCurrentPosition());
        telemetry.addData("LBA1", LBA.DC1.getCurrentPosition());
        telemetry.addData("LBA2", LBA.DC2.getCurrentPosition());
        telemetry.addData("Reset", "Done");

        telemetry.update();

    }

    @Override
    protected void run(double dt) throws InterruptedException {
        //Control

        telemetry.addData("you", "are not supposed to be here");
        telemetry.update();
    }
}
