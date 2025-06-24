package org.firstinspires.ftc.teamcode.pathing.follower

import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.min

class DriveConstants {
    companion object {
        /** The maximum velocity of the drive motors in ticks per second. */
        const val MAX_DRIVE_MOTOR_VELOCITY = 2500.0

        /** The default power to use when no other power is specified. */
        const val DEFAULT_DRIVE_POWER = 0.7

        /** The maximum forward velocity in cm per second. */
        const val MAX_FORWARD_VELOCITY = 150.0

        /** The maximum horizontal velocity in cm per second. */
        const val MAX_HORIZONTAL_VELOCITY = 125.0

        /** The maximum drive velocity in cm per second. */
        val MAX_DRIVE_VELOCITY = min(MAX_FORWARD_VELOCITY, MAX_HORIZONTAL_VELOCITY)

        /** Look ahead distance in cm for the Pure Pursuit algorithm. */
        const val LOOK_AHEAD_DISTANCE = 15.0

        /** The threshold in cm to consider the target reached.*/
        const val TARGET_REACHED_THRESHOLD = 1.0

        /** The directions of the drive motors. */
        val MOTOR_DIRECTIONS = arrayOf(
            DcMotorSimple.Direction.REVERSE, // Left Front
            DcMotorSimple.Direction.REVERSE, // Left Back
            DcMotorSimple.Direction.REVERSE, // Right Front
            DcMotorSimple.Direction.FORWARD, // Right Back
        )
    }
}
