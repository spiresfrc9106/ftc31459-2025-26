package org.firstinspires.ftc.teamcode.pathing.follower

import com.qualcomm.robotcore.hardware.DcMotorSimple

class DriveConstants {
    companion object {
        /** The maximum velocity of the drive motors in ticks per second. */
        const val MAX_DRIVE_MOTOR_VELOCITY = 2500.0

        /** The default power to use when no other power is specified. */
        const val DEFAULT_DRIVE_POWER = 0.7

        /** The maximum forward velocity in cm per second. */
        const val MAX_FORWARD_VELOCITY = 50.0 // Needs to be tuned

        /** The maximum horizontal velocity in cm per second. */
        const val MAX_HORIZONTAL_VELOCITY = 50.0 // Needs to be tuned

        /** The directions of the drive motors. */
        val MOTOR_DIRECTIONS = arrayOf(
            DcMotorSimple.Direction.REVERSE, // Left Front
            DcMotorSimple.Direction.REVERSE, // Left Back
            DcMotorSimple.Direction.REVERSE, // Right Front
            DcMotorSimple.Direction.FORWARD, // Right Back
        )
    }
}
