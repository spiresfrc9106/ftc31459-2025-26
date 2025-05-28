package org.firstinspires.ftc.teamcode.localization.constants

import org.firstinspires.ftc.teamcode.localization.localizers.GoBildaPinpointDriver

class PinpointConstants {
    companion object {
        /**
         * How far sideways (in mm) from the tracking point the Y (forward) odometry pod is.
         * Left increases.
         */
        const val Y_POD_OFFSET = 0.0

        /**
         * How far forwards (in mm) from the tracking point the X (strafe) odometry pod is.
         * Forward increases.
         */
        const val X_POD_OFFSET = -134.0

        /** The direction of the Y (forward) odometry pod. It should increase when moving forward. */
        val Y_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD

        /** The direction of the X (strafe) odometry pod. It should increase when strafing right. */
        val X_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED

        /** The encoder resolution for the odometry pods. */
        val ENCODER_RESOLUTION = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
    }
}
