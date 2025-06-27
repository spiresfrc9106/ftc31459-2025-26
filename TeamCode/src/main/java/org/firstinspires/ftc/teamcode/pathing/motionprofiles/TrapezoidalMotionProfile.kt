package org.firstinspires.ftc.teamcode.pathing.motionprofiles

import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import kotlin.math.sqrt

class TrapezoidalMotionProfile(private val totalDist: Double) : MotionProfile {
    private val maxAccel: Double = DriveConstants.MAX_DRIVE_ACCELERATION
    private val maxVel: Double = DriveConstants.MAX_DRIVE_VELOCITY

    private val accelDist: Double
    private val decelDist: Double
    private val cruiseDist: Double
    private val peakVel: Double
    private val isTriangular: Boolean

    init {
        val minDistToReachMaxVel = (maxVel * maxVel) / maxAccel

        isTriangular = totalDist < minDistToReachMaxVel

        if (isTriangular) {
            // Can't reach maxVel, so compute peak velocity for triangular profile
            peakVel = sqrt(maxAccel * totalDist)
            accelDist = totalDist / 2
            decelDist = totalDist / 2
            cruiseDist = 0.0
        } else {
            peakVel = maxVel
            accelDist = (maxVel * maxVel) / (2 * maxAccel)
            decelDist = accelDist
            cruiseDist = totalDist - accelDist - decelDist
        }
    }

    override fun getVelocity(s: Double): Double {
        return when {
            s < accelDist -> sqrt(2 * maxAccel * s)
            s < accelDist + cruiseDist -> peakVel
            s < totalDist -> sqrt(2 * maxAccel * (totalDist - s))
            else -> 0.0
        }
    }
}
