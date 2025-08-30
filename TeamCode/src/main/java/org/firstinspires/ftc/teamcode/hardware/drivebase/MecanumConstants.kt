package org.firstinspires.ftc.teamcode.hardware.drivebase

import com.qualcomm.robotcore.hardware.DcMotorSimple

class MecanumConstants {
    companion object {
        /** The track width of the robot in cm. (distance between the left and right wheels) */
        const val TRACK_WIDTH = 0.0

        /** The wheel base of the robot in cm. (distance between the front and back wheels) */
        const val WHEEL_BASE = 0.0

        /** The maximum velocity of the drive motors in ticks per second. */
        const val MAX_DRIVE_MOTOR_VELOCITY = 2500.0

        /** The default power to use when no other power is specified. */
        const val DEFAULT_DRIVE_POWER = 0.7

        // Horizontal and vertical velocities used to calculate strafe power adjustments
        /** The maximum forward velocity in cm per second. */
        const val MAX_FORWARD_VELOCITY = 150.0

        /** The maximum horizontal velocity in cm per second. */
        const val MAX_HORIZONTAL_VELOCITY = 125.0

        const val KV = 0.0 // Velocity constant for motor feedforward
        const val KA = 0.0 // Acceleration constant for motor feedforward
        const val KS = 0.0 // Static friction constant for motor feedforward

        /** The directions of the drive motors. */
        val MOTOR_DIRECTIONS = arrayOf(
            DcMotorSimple.Direction.REVERSE, // Left Front
            DcMotorSimple.Direction.REVERSE, // Left Back
            DcMotorSimple.Direction.FORWARD, // Right Front
            DcMotorSimple.Direction.FORWARD, // Right Back
        )
    }
}