package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;


@FunctionalInterface
interface UserCommandsIntake {
    Boolean userWantsIntakeToRun();
}

@FunctionalInterface
interface UserCommandsPushOutOfIntake {
    Boolean userWantsPushOutOfIntake();
}
@Config
public final class ExampleIntake {
    public static class Params {
        public double todoAddParamsHere = 0.04024;
    }

    public static Params PARAMS = new Params();


    private UserCommandsIntake userCommandIntake;
    private UserCommandsPushOutOfIntake userCommandsPushOutOfIntake;
    private DcMotorEx motor;

    TouchSensor touchSensor;

    private double targetVelocityRPS = 0.0;
    private double curVelocityRPS = 0.0;
    public static double intakingVelocityRPS = 6.0;
    public static double TICKS_PER_REVOLUTION = 537.7;

    private boolean prevUserCommand = false;

    public enum IntakeStates {
        IDLE, INTAKING, PAUSED_FULL
    }

    private IntakeStates state = IntakeStates.IDLE;



    public ExampleIntake(HardwareMap hardwareMap, UserCommandsIntake givenIntakeUserCommand, UserCommandsPushOutOfIntake givenPushOutOfIntakeCommand) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        userCommandIntake = givenIntakeUserCommand;
        userCommandsPushOutOfIntake = givenPushOutOfIntakeCommand;

        motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motor.setVelocity(0.0);

        touchSensor = hardwareMap.get(TouchSensor.class, "intake_touch");

    }


    public void sendPlotData(@NonNull TelemetryPacket p) {
        p.put("intake_motor_target_RPS", targetVelocityRPS);
        p.put("intake_motor_current_RPS", curVelocityRPS);
    }


    public class PerhapsMoveIntakeTeleOp implements Action {

        public PerhapsMoveIntakeTeleOp() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            double targetTicksPerSecond = 0.0;

            switch (state) {
                case IDLE:
                    if (touchSensor.isPressed()) {
                        state = IntakeStates.PAUSED_FULL;
                        targetVelocityRPS = 0.0;
                    } else if (userCommandIntake.userWantsIntakeToRun()) {
                        targetVelocityRPS = intakingVelocityRPS;
                        state = IntakeStates.INTAKING;

                    } else {
                        targetVelocityRPS = 0.0;
                    }
                    break;
                case INTAKING: {
                    if (touchSensor.isPressed()) {
                        state = IntakeStates.PAUSED_FULL;
                        targetVelocityRPS = 0.0;
                    } else if (userCommandIntake.userWantsIntakeToRun()) {
                        targetVelocityRPS = targetVelocityRPS;

                    } else {
                        targetVelocityRPS = 0.0;
                        state = IntakeStates.IDLE;
                    }
                }
                case PAUSED_FULL: {
                    if (userCommandsPushOutOfIntake.userWantsPushOutOfIntake()) {
                        targetVelocityRPS = intakingVelocityRPS;
                    }
                    if (!touchSensor.isPressed() && !userCommandsPushOutOfIntake.userWantsPushOutOfIntake()) {
                        targetVelocityRPS = 0.0;
                        state = IntakeStates.IDLE;
                    }
                    
                }

                    
            }

 

            targetTicksPerSecond = targetVelocityRPS * TICKS_PER_REVOLUTION;

            motor.setVelocity(targetTicksPerSecond);

            double velocityTicksPerSecond = motor.getVelocity();
            curVelocityRPS = velocityTicksPerSecond / TICKS_PER_REVOLUTION;

            return true; // In teleop, the true keeps this action on the action list
        }
    }

}
