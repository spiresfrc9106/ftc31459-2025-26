package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

public class TurretSubsystem extends SubsystemBase {
    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo leftPivot;
    Servo rightPivot;
    double servoRange = 180;
    public TurretSubsystem(DcMotor leftWheel, DcMotor rightWheel, Servo leftPivot, Servo rightPivot) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        this.leftPivot = leftPivot;
        this.rightPivot = rightPivot;
    }

    Vector2d Position(double time, Vector2d speed, double a) {
        double px = (Constants.mass / Constants.xCoefficient) * Math.log1p(speed.getX() * time * Constants.xCoefficient / Constants.mass);
        double py = (-Math.pow(Constants.terminalVelocity, 2) / Constants.gravity) * Math.log(Math.cosh(a - time * Constants.yCoefficient) / Math.cosh(a));
        return new Vector2d(px, py);
    }
    Vector2d PositionDTheta(double time, Vector2d speed, double a) {
        double px = (time * Constants.mass * speed.getY()) / (speed.getX() * time * Constants.xCoefficient + Constants.mass);
        double py = (speed.getX() / (Constants.gravity - Constants.xCoefficient / Constants.mass * Math.pow(speed.getY(), 2))) * (speed.getY() - Constants.terminalVelocity * Math.tanh(a - Constants.yCoefficient * time));
        return new Vector2d(px, py);
    }

    /** @noinspection SpellCheckingInspection*/
    double atanh(double x) {
        return 0.5 * Math.log((1 + x) / (1 - x));
    }

    double ThetaEstimate(double target) {
        Vector2d speed = new Vector2d(Constants.speed * Math.cos(Constants.theta0), Constants.speed * Math.sin(Constants.theta0));
        double a;
        if (speed.getY() > Constants.terminalVelocity) a =  Constants.terminalVelocity - 0.001;
        else a = atanh(speed.getY() / Constants.terminalVelocity);
        double x0 = 2 * a / Constants.yCoefficient;
        Vector2d dGuess = Position(x0, speed, a);;
        double thetaGuess = Constants.theta0;

        while(dGuess.getX() - target > Constants.toleranceThreshold) {
            // Estimate
            double deltaD = dGuess.getX() - target;
            thetaGuess = thetaGuess - (deltaD / PositionDTheta(x0, speed, a).getX());

            // Update
            speed = new Vector2d(Constants.speed * Math.cos(thetaGuess), Constants.speed * Math.sin(thetaGuess));
            if (speed.getY() > Constants.terminalVelocity) a =  Constants.terminalVelocity - 0.001;
            else a = atanh(speed.getY() / Constants.terminalVelocity);
            dGuess = Position(x0, speed, a);
        }

        return thetaGuess;
    }

    public boolean AimTurret(double target) {
        double theta = ThetaEstimate(target);
        double servoPosition = theta / servoRange; //TODO: Change to make in range 35, 90 inclusive because we love inclusivity :)
        leftPivot.setPosition(servoPosition);
        rightPivot.setPosition(1 - servoPosition); //TODO: Figure out which is opposite
        return true; //TODO: Return some value to ensure its aimed and the target is reachable
    }

    public void FireTurret() {
        leftWheel.setPower(1); //TODO: Figure out which is opposite
        rightWheel.setPower(-1);
    }

    public void StopTurret() {
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
}
