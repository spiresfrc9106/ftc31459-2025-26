package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.HardwareNames
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import kotlin.math.abs

class MecanumBase (hardwareMap: HardwareMap) {
    // Drive motors
    val leftFront = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_LEFT_FRONT)
    val leftBack = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_LEFT_BACK)
    val rightFront = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_RIGHT_FRONT)
    val rightBack = hardwareMap.get(DcMotorEx::class.java, HardwareNames.DRIVE_RIGHT_BACK)
    val motors = arrayOf(leftFront, leftBack, rightFront, rightBack)

    init {
        // Initialize motors
        for ((i, motor) in motors.withIndex()) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.direction = DriveConstants.MOTOR_DIRECTIONS[i]
        }
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
     */
    fun moveVector(x: Double, y: Double, rotation: Double, power: Double = DriveConstants.DEFAULT_DRIVE_POWER, adjustForStrafe: Boolean = true) {
        // Scale movement vector based on the maximum velocity
        val adjustedX = if (adjustForStrafe) (x * DriveConstants.MAX_HORIZONTAL_VELOCITY / DriveConstants.MAX_FORWARD_VELOCITY) else x
        val adjustedY = y

        val denominator = (abs(adjustedY) + abs(adjustedX) + abs(rotation)).coerceAtLeast(1.0)
        leftFront.velocity = ((adjustedY + adjustedX + rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        leftBack.velocity = ((adjustedY - adjustedX + rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        rightFront.velocity = ((adjustedY - adjustedX - rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
        rightBack.velocity = ((adjustedY + adjustedX - rotation) / denominator) * power * DriveConstants.MAX_DRIVE_MOTOR_VELOCITY
    }

    /**
     * Sets the power of all drive motors to the 0.0
     */
    fun stop() {
        for (motor in motors) {
            motor.power = 0.0
        }
    }
}
