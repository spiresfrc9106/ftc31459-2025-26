package org.firstinspires.ftc.teamcode.pathing.follower

import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.motionprofilers.MotionProfile
import org.firstinspires.ftc.teamcode.pathing.paths.Path
import kotlin.math.sqrt

class Follower {
    private val TAG = "Follower"
    var path: Path? = null
    var motionProfile: MotionProfile? = null
    var lookaheadPoint = Pose()

    /**
     * Update drive powers to follow the path using Pure Pursuit.
     * Assumes that the path is already set and that the pose has been updated
     */
    fun update() {
        if (path == null) {
            FileLogger.error(TAG, "Path is null")
            return
        }
        val lookaheadPointT = path!!.getLookaheadPointT(Bot.localizer.pose, DriveConstants.LOOK_AHEAD_DISTANCE)
        lookaheadPoint = path!!.getPoint(lookaheadPointT)

        // Move towards the lookahead point
        val dx = lookaheadPoint.x - Bot.localizer.pose.x
        val dy = lookaheadPoint.y - Bot.localizer.pose.y
        val distance = sqrt(dx * dx + dy * dy)

        // TODO: Calculate the target rotation speed
        val targetSpeed = getTargetSpeed(lookaheadPointT)
        val targetRotationSpeed = 0.0 // Radians per second

        // TODO: Convert global coordinates dx, dy to local coordinates (use utility function?)
        // Works for now if the robot is always facing forward

        Bot.mecanumBase.moveVelocity(dx / distance * targetSpeed, dy / distance * targetSpeed, targetRotationSpeed)
    }

    fun reachedTarget(): Boolean {
        if (path == null) {
            FileLogger.error(TAG, "Path is null")
            return false
        }
        val goal = path!!.endPose
        return goal.roughlyEquals(
            Bot.localizer.pose,
            DriveConstants.TARGET_REACHED_THRESHOLD
        )
    }

    fun getTargetSpeed(t: Double): Double {
        if (motionProfile == null) {
            return DriveConstants.MAX_DRIVE_VELOCITY
        }
        // TODO: Implement a motion profile
        return motionProfile!!.getVelocity(t).coerceAtMost(DriveConstants.MAX_DRIVE_VELOCITY)
    }
}
