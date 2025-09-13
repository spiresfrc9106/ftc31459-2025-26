package org.firstinspires.ftc.teamcode.Subsytems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.smartcluster.oracleftc.commands.Command;
import com.smartcluster.oracleftc.commands.helpers.InstantCommand;
import com.smartcluster.oracleftc.hardware.Subsystem;
import com.smartcluster.oracleftc.hardware.SubsystemFlavor;

import java.util.HashSet;
import java.util.Set;

public class Intake extends Subsystem {
    private DcMotorEx intake = null;
    private static double[]power={0,0.9,0.8};//0->stop 1->intake 2->outtake


    public Intake(OpMode opMode) {
        super(opMode);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public SubsystemFlavor flavor() {
        return SubsystemFlavor.ControlHubOnly;
    }

    public Command stop() {return new InstantCommand(() -> {intake.setPower(power[0]);});}
    public Command intake() {return new InstantCommand(()->intake.setPower(power[1]));}
    public Command outtake() {return new InstantCommand(()->intake.setPower(power[2]));}


    public Set<Subsystem> requires() {
        Set<Subsystem> set = new HashSet<Subsystem>();
        set.add(Intake.this);
        return set;
    }
    public Command telemetry()
    {
        return Command.builder()
                .update(()->{
                    telemetry.addData("Intake power:", intake.getPower());
                })
                .requires(this)
                .build();
    }
    public static class Manual {
        public double intakePower=0.0;
    };

    public static Manual manual = new Manual();

    public Command manual()
    {
        return Command.builder()
                .update(()->{
                    intake.setPower(manual.intakePower);
                })
                .requires(this)
                .build();
    }
}
