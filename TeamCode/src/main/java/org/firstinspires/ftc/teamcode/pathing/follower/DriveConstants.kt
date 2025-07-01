package org.firstinspires.ftc.teamcode.pathing.follower

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Config
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

        /** The maximum rotational velocity in radians per second. */
        const val MAX_ROTATIONAL_VELOCITY = 3.0

        /** The maximum drive velocity in cm per second. */
        const val MAX_DRIVE_VELOCITY = 100.0

        /** The minimum drive velocity in cm per second. */
        const val MIN_DRIVE_VELOCITY = 5.0

        /** The maximum drive acceleration in cm per second squared. */
        const val MAX_DRIVE_ACCELERATION = 50.0

        /** The maximum centripetal acceleration that the robot can handle in cm/s^2 **/
        const val MAX_CENTRIPETAL_ACCELERATION = (70.0 * 70.0) / 25.0

        /** Look ahead distance in cm for the Pure Pursuit algorithm. */
        const val LOOK_AHEAD_DISTANCE = 15.0

        /** The threshold in cm to consider the target reached.*/
        const val POSITION_THRESHOLD = 0.5

        /** The threshold in degrees to consider the target reached.*/
        const val ROTATION_THRESHOLD = 0.01

        @JvmField var PIDF_X = arrayOf(
            0.015,   // P
            0.0,    // I
            0.0025,  // D
            0.006   // F
        )

        @JvmField var PIDF_Y = arrayOf(
            0.015,  // P
            0.0,    // I
            0.0025,  // D
            0.005   // F
        )

        @JvmField var PIDF_OMEGA = arrayOf(
            0.1,   // P
            0.0,   // I
            0.01,  // D
            0.18   // F
        )

        @JvmField var PID_ROTATION = arrayOf(
            5.0,  // P
            0.05, // I
            25.0, // D
        )

        /** The directions of the drive motors. */
        val MOTOR_DIRECTIONS = arrayOf(
            DcMotorSimple.Direction.REVERSE, // Left Front
            DcMotorSimple.Direction.REVERSE, // Left Back
            DcMotorSimple.Direction.FORWARD, // Right Front
            DcMotorSimple.Direction.FORWARD, // Right Back
        )
    }
}
