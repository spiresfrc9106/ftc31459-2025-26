package org.firstinspires.ftc.teamcode.pathing.follower

import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger
import org.firstinspires.ftc.teamcode.helpers.PIDController
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.motionprofiles.MotionProfile
import org.firstinspires.ftc.teamcode.pathing.motionprofiles.TrapezoidalMotionProfile
import org.firstinspires.ftc.teamcode.pathing.paths.Path
import kotlin.math.sqrt

class Follower {
    private val TAG = "Follower"

    var path: Path? = null
        set(value) {
            field = value
            // Reset the motion profile when the path changes
            motionProfile = if (path == null) null else TrapezoidalMotionProfile(path!!.getLength())
        }
    var motionProfile: MotionProfile? = null
    var lookaheadPointT = 0.0
    var lookaheadPoint = Pose()

    val rotationPID = PIDController(DriveConstants.PID_ROTATION)

    /**
     * Update drive powers to follow the path using Pure Pursuit.
     * Assumes that the path is already set and that the pose has been updated
     */
    fun update() {
        if (path == null) {
            return
        }
        // Get the lookahead point on the path (default to 0.0 if not found)
        lookaheadPointT = path!!.getLookaheadPointT(Bot.localizer.pose, DriveConstants.LOOK_AHEAD_DISTANCE) ?: 0.0
        lookaheadPoint = path!!.getPoint(lookaheadPointT)

        // Move towards the lookahead point
        val dx = lookaheadPoint.x - Bot.localizer.pose.x
        val dy = lookaheadPoint.y - Bot.localizer.pose.y
        val distance = sqrt(dx * dx + dy * dy)

        val targetSpeed = getTargetSpeed()

        // Calculate target rotational velocity using PID
        var targetRotationSpeed = rotationPID.update(lookaheadPoint.heading, Bot.localizer.pose.heading, Bot.dt, normalizeRadians = true)
        targetRotationSpeed = targetRotationSpeed.coerceIn(-DriveConstants.MAX_ROTATIONAL_VELOCITY, DriveConstants.MAX_ROTATIONAL_VELOCITY)

        // Convert global coordinates dx, dy to local coordinates
        val difference = Pose(dx, dy)
        difference.rotate(Bot.localizer.pose.heading)

        Bot.mecanumBase.moveVelocity(difference.x / distance * targetSpeed, difference.y / distance * targetSpeed, targetRotationSpeed)
    }

    fun reachedTarget(): Boolean {
        if (path == null) {
            FileLogger.error(TAG, "Path is null")
            return false
        }
        val goal = path!!.endPose
        return goal.roughlyEquals(
            Bot.localizer.pose,
            DriveConstants.POSITION_THRESHOLD,
            DriveConstants.ROTATION_THRESHOLD
        )
    }

    fun getTargetSpeed(): Double {
        if (motionProfile == null) {
            val slowDownDistance = DriveConstants.LOOK_AHEAD_DISTANCE * 2.0
            val scale = Bot.localizer.pose.distanceTo(path!!.endPose) / slowDownDistance
            val velocityScale = scale.coerceIn(0.0, 1.0)
            return (DriveConstants.MAX_DRIVE_VELOCITY * 0.5 * velocityScale).coerceAtLeast(DriveConstants.MIN_DRIVE_VELOCITY)
        }
        val closestPointT = path!!.getClosestPointT(Bot.localizer.pose)
        val distanceTraveled = path!!.getLengthSoFar(closestPointT)
        return motionProfile!!.getVelocity(distanceTraveled).coerceIn(DriveConstants.MIN_DRIVE_VELOCITY, DriveConstants.MAX_DRIVE_VELOCITY)
    }
}
