package org.firstinspires.ftc.teamcode.helpers

import kotlin.math.PI

class Angle {
    companion object {
        /**
         * Normalizes an angle to the range (-π, π].
         * @param angle The angle in radians to normalize.
         * @return The normalized angle in radians.
         */
        fun normalizeRadians(angle: Double): Double {
            var normalized = angle
            while (normalized > PI) normalized -= 2 * PI
            while (normalized <= -PI) normalized += 2 * PI
            return normalized
        }
    }
}
