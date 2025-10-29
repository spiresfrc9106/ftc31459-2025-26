package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public final class ExampleIntake {
    public static class Params {
        public double todoAddParamsHere = 0.04024;
    }

    public static Params PARAMS = new Params();



    private DcMotorEx motor;

    private double targetVelocityRPS = 0.0;
    public static double intakingVelocityRPS = 6.0;
    public static double TICKS_PER_REVOLUTION = 537.7;



    public ExampleIntake(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motor.setVelocity(0.0);
        int curPosition = motor.getCurrentPosition();

        double velocityTicksPerSecond = motor.getVelocity();
        double curVelocityRPS = velocityTicksPerSecond / TICKS_PER_REVOLUTION;
    }


    public void sendPlotData(@NonNull TelemetryPacket p) {
        p.put("intake_motor", targetVelocityRPS);
    }


    public class MoveServo implements Action {
        double newTargetPosition;

        public MoveServo(Servo servo, double newTargetPosition) {
            this.newTargetPosition = newTargetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetVelocityRPS = this.newTargetPosition;
            // TODO Fix me motor.setPosition(targetVelocityRPS);
            return false;
        }
    }

    public class MoveServoPatient implements Action {

        double newTargetPosition;
        double waitTime_s;
        ElapsedTime timer;
        public MoveServoPatient(Servo servo, double newTargetPosition, double waitTime_s) {
            this.newTargetPosition = targetVelocityRPS;
            this.waitTime_s = waitTime_s;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer==null) {
                timer = new ElapsedTime();
            }
            targetVelocityRPS = this.newTargetPosition;
            //TODO Fixme motor.setPosition(targetVelocityRPS);
            return timer.seconds() < waitTime_s;
        }

    }


}
