package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



@Config
public final class ExampleHardware {
    public static class Params {
        public double todoAddParamsHere = 0.04024;
    }

    public static Params PARAMS = new Params();


    private Servo servo;

    private double targetPosition = 0;


    public ExampleHardware(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        Servo servo = hardwareMap.servo.get("servo0");


    }


    public void sendPlotData(@NonNull TelemetryPacket p) {
        p.put("servo0", targetPosition);
    }


    public class MoveServo implements Action {
        double newTargetPosition;

        public MoveServo(Servo servo, double newTargetPosition) {
            this.newTargetPosition = newTargetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = this.newTargetPosition;
            servo.setPosition(targetPosition);
            return false;
        }
    }

    public class MoveServoPatient implements Action {

        double newTargetPosition;
        double waitTime_s;
        ElapsedTime timer;
        public MoveServoPatient(Servo servo, double newTargetPosition, double waitTime_s) {
            this.newTargetPosition = targetPosition;
            this.waitTime_s = waitTime_s;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer==null) {
                timer = new ElapsedTime();
            }
            targetPosition = this.newTargetPosition;
            servo.setPosition(targetPosition);
            return timer.seconds() < waitTime_s;
        }

    }


}
