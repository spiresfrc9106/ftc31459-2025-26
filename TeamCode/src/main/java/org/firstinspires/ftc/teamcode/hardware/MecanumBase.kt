package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.HardwareNames
import org.firstinspires.ftc.teamcode.helpers.PIDFController
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import kotlin.math.abs

class MecanumBase (hardwareMap: HardwareMap) {
    // Drive motors
    val leftFront = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_LEFT_FRONT)
    val leftBack = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_LEFT_BACK)
    val rightFront = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_RIGHT_FRONT)
    val rightBack = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_RIGHT_BACK)
    val motors = arrayOf(leftFront, leftBack, rightFront, rightBack)

    // PIDF controllers for velocity control
    // TODO: Tune velocity PIDF controllers
    private val pidX = PIDFController(0.0, 0.0, 0.0, 0.0)
    private val pidY = PIDFController(0.0, 0.0, 0.0, 0.0)
    private val pidOmega = PIDFController(0.0, 0.0, 0.0, 0.0)

    init {
        // Initialize motors
        for (motor in motors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        leftFront.direction = DriveConstants.MOTOR_DIRECTIONS[0]
        leftBack.direction = DriveConstants.MOTOR_DIRECTIONS[1]
        rightFront.direction = DriveConstants.MOTOR_DIRECTIONS[2]
        rightBack.direction = DriveConstants.MOTOR_DIRECTIONS[3]
    }

    /**
     * Sets the zero power behavior of all drive motors.
     */
    fun setZeroPowerBehavior(behavior: DcMotor.ZeroPowerBehavior) {
        for (motor in motors) {
            motor.zeroPowerBehavior = behavior
        }
    }

    /**
     * Sets the power of all drive motors to move in a specific direction.
     * @param x The x component of the movement vector (right is positive).
     * @param y The y component of the movement vector (forward is positive).
     * @param rotation The rotation component (counter-clockwise is positive).
     * @param power Scaling factor for the drive power, default is [DriveConstants.DEFAULT_DRIVE_POWER].
     * @param adjustForStrafe Whether to adjust the x component for strafing inefficiency.
     */
    fun moveVector(x: Double, y: Double, rotation: Double, power: Double = DriveConstants.DEFAULT_DRIVE_POWER, adjustForStrafe: Boolean = true) {
        // Adjust x for strafing if necessary
        val strafeFactor = DriveConstants.MAX_FORWARD_VELOCITY / DriveConstants.MAX_HORIZONTAL_VELOCITY
        val adjX = if (adjustForStrafe) x * strafeFactor else x

        val denominator = (abs(y) + abs(adjX) + abs(rotation)).coerceAtLeast(1.0)
        leftFront.velocity = ((y + adjX + rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        leftBack.velocity = ((y - adjX + rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        rightFront.velocity = ((y - adjX - rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        rightBack.velocity = ((y + adjX - rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
    }

    /**
     * Uses PIDF to move the robot in a direction specified by velocity components.
     * @param vx The x component of the velocity in cm/s (right is positive).
     * @param vy The y component of the velocity in cm/s (forward is positive).
     * @param omega The angular velocity in radians/second (clockwise is positive).
     */
    fun moveVelocity(vx: Double, vy: Double, omega: Double) {
        val xPower = pidX.update(vx, Bot.localizer.velocity.x, Bot.dt).coerceIn(-1.0, 1.0)
        val yPower = pidY.update(vy, Bot.localizer.velocity.y, Bot.dt).coerceIn(-1.0, 1.0)
        val omegaPower = pidOmega.update(omega, Bot.localizer.velocity.heading, Bot.dt).coerceIn(-1.0, 1.0)
        moveVector(xPower, yPower, omegaPower, 1.0, false)
    }

    /**
     * Sets the power of all drive motors to the 0.0
     */
    fun stop() {
        for (motor in motors) {
            motor.velocity = 0.0
        }
    }
}
