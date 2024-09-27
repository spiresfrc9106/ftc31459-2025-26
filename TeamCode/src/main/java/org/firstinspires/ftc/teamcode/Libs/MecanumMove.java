package org.firstinspires.ftc.teamcode.Libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;

public class MecanumMove {
    public DcMotor wheel1, wheel2, wheel3, wheel4;
    //Wheel 1 is top left, wheel 2 is top right, wheel 3 is bottom left, wheel 4 is bottom right


    public void sleep(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void TestW4 (double power) {
        wheel4.setPower(power);
    }


    public void moveforward(double power) {
        wheel1.setPower(power);
        wheel2.setPower(power);
        wheel3.setPower(power);
        wheel4.setPower(power);
    }

    public void movebackward(double power) {
        wheel1.setPower(-power);
        wheel2.setPower(-power);
        wheel3.setPower(-power);
        wheel4.setPower(-power);
    }

    public void turnleft(double power) {
        wheel1.setPower(-power);
        wheel2.setPower(power);
        wheel3.setPower(-power);
        wheel4.setPower(power);
    }

    public void turnright(double power) {
        wheel1.setPower(power);
        wheel2.setPower(-power);
        wheel3.setPower(power);
        wheel4.setPower(-power);
    }

    public void stopmoving() {
        wheel1.setPower(0);
        wheel2.setPower(0);
        wheel3.setPower(0);
        wheel4.setPower(0);
    }

    public void moveleft(double power) {
        wheel1.setPower(-power);
        wheel2.setPower(power);
        wheel3.setPower(power);
        wheel4.setPower(-power);
    }

    public void moveright(double power) {
        wheel1.setPower(power);
        wheel2.setPower(-power);
        wheel3.setPower(-power);
        wheel4.setPower(power);
    }

    public void moveforward(double power, int distance) {
        wheel1.setTargetPosition(distance);
        wheel2.setTargetPosition(distance);
        wheel3.setTargetPosition(distance);
        wheel4.setTargetPosition(distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveforward(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void movebackward(double power, int distance) {
        wheel1.setTargetPosition(-distance);
        wheel2.setTargetPosition(-distance);
        wheel3.setTargetPosition(-distance);
        wheel4.setTargetPosition(-distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        movebackward(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnleft(double power, int distance) {
        wheel1.setTargetPosition(-distance);
        wheel2.setTargetPosition(distance);
        wheel3.setTargetPosition(-distance);
        wheel4.setTargetPosition(distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnleft(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnright(double power, int distance) {
        wheel1.setTargetPosition(distance);
        wheel2.setTargetPosition(-distance);
        wheel3.setTargetPosition(distance);
        wheel4.setTargetPosition(-distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnright(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void strafeleft (double power, int distance) {
        wheel1.setTargetPosition(-distance);
        wheel2.setTargetPosition(distance);
        wheel3.setTargetPosition(distance);
        wheel4.setTargetPosition(-distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveleft(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void straferight (double power, int distance) {
        wheel1.setTargetPosition(distance);
        wheel2.setTargetPosition(-distance);
        wheel3.setTargetPosition(-distance);
        wheel4.setTargetPosition(distance);
        wheel1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheel4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveright(power);
        while (wheel1.isBusy() && wheel2.isBusy() && wheel3.isBusy() && wheel4.isBusy()) {
            sleep(10);
        }
        stopmoving();
        wheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheel4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void move(double x, double y, double turn) {
        wheel1.setPower(y + x + turn);
        wheel2.setPower(y - x - turn);
        wheel3.setPower((y - x + turn);
        wheel4.setPower((y + x - turn);
    }

    public void moveV(Vector3 vector3, double power) {
        move(vector3.x * power, vector3.y * power, vector3.z * power);
    }


    public void Gmove(double x, double y, double Turn, double currentAngle) {
        //Turns the x and y into an angle
        double angle = Math.atan2(y, x); //Is this radians or degrees? : Radians

        //Convert to Current Angle to Radians
        double RadCA = Math.toRadians(currentAngle);
        RadCA *= -1;
        //Get new angle
        double newAngle = -RadCA + angle;

        //Get new x and y
        double newX = Math.cos(newAngle) * Math.sqrt(x * x + y * y);
        double newY = Math.sin(newAngle) * Math.sqrt(x * x + y * y);

        //Move
        move(newX, newY, Turn);

    }

}
