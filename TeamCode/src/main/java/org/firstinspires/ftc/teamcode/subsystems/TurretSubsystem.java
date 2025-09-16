package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class TurretSubsystem extends SubsystemBase {
    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo leftPivot;
    Servo rightPivot;
    double servoRange = 180;
    public TurretSubsystem(HardwareMap hardwareMap) {
        this.leftWheel = hardwareMap.get(DcMotor.class, Constants.HardwareMap.leftTurretWheelName);
        this.rightWheel = hardwareMap.get(DcMotor.class, Constants.HardwareMap.rightTurretWheelName);
        this.leftPivot = hardwareMap.get(Servo.class, Constants.HardwareMap.leftTurretPivotName);
        this.rightPivot = hardwareMap.get(Servo.class, Constants.HardwareMap.rightTurretPivotName);
    }

    Vector2d position(double time, Vector2d speed, double a) {
        double px = (Constants.Physics.mass / Constants.Physics.xCoefficient) * Math.log1p(speed.getX() * time * Constants.Physics.xCoefficient / Constants.Physics.mass);
        double py = (-Math.pow(Constants.Physics.terminalVelocity, 2) / Constants.Physics.gravity) * Math.log(Math.cosh(a - time * Constants.Physics.yCoefficient) / Math.cosh(a));
        return new Vector2d(px, py);
    }
    Vector2d positionDTheta(double time, Vector2d speed, double a) {
        double px = (time * Constants.Physics.mass * speed.getY()) / (speed.getX() * time * Constants.Physics.xCoefficient + Constants.Physics.mass);
        double py = (speed.getX() / (Constants.Physics.gravity - Constants.Physics.xCoefficient / Constants.Physics.mass * Math.pow(speed.getY(), 2))) * (speed.getY() - Constants.Physics.terminalVelocity * Math.tanh(a - Constants.Physics.yCoefficient * time));
        return new Vector2d(px, py);
    }

    /** @noinspection SpellCheckingInspection*/
    double atanh(double x) {
        return 0.5 * Math.log((1 + x) / (1 - x));
    }

    double thetaEstimate(double target) {
        Vector2d speed = new Vector2d(Constants.Physics.speed * Math.cos(Constants.Physics.theta0), Constants.Physics.speed * Math.sin(Constants.Physics.theta0));
        double a;
        if (speed.getY() > Constants.Physics.terminalVelocity) a =  Constants.Physics.terminalVelocity - 0.001;
        else a = atanh(speed.getY() / Constants.Physics.terminalVelocity);
        double x0 = 2 * a / Constants.Physics.yCoefficient;
        Vector2d dGuess = position(x0, speed, a);;
        double thetaGuess = Constants.Physics.theta0;

        while(dGuess.getX() - target > Constants.Physics.toleranceThreshold) {
            // Estimate
            double deltaD = dGuess.getX() - target;
            thetaGuess = thetaGuess - (deltaD / positionDTheta(x0, speed, a).getX());

            // Update
            speed = new Vector2d(Constants.Physics.speed * Math.cos(thetaGuess), Constants.Physics.speed * Math.sin(thetaGuess));
            if (speed.getY() > Constants.Physics.terminalVelocity) a =  Constants.Physics.terminalVelocity - 0.001;
            else a = atanh(speed.getY() / Constants.Physics.terminalVelocity);
            dGuess = position(x0, speed, a);
        }

        return thetaGuess;
    }

    public boolean aimTurret(double target) {
        double theta = thetaEstimate(target);
        double servoPosition = theta / servoRange; //TODO: Change to make in range 35, 90 inclusive because we love inclusivity :)
        leftPivot.setPosition(servoPosition);
        rightPivot.setPosition(1 - servoPosition); //TODO: Figure out which is opposite
        return true; //TODO: Return some value to ensure its aimed and the target is reachable
    }

    public void fireTurret() {
        leftWheel.setPower(1); //TODO: Figure out which is opposite
        rightWheel.setPower(-1);
    }

    public void stopTurret() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }

    public boolean isTurretMoving() {
        return leftWheel.getPower() != 0 || rightWheel.getPower() != 0;
    }
}
