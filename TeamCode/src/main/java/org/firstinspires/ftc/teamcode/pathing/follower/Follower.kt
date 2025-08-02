package org.firstinspires.ftc.teamcode.pathing.follower

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger
import org.firstinspires.ftc.teamcode.helpers.PIDController
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionProfile
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionProfileGenerator
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionState
import org.firstinspires.ftc.teamcode.pathing.paths.Path
import kotlin.math.*

class Follower {
    private var motionProfile: MotionProfile? = null
    private var elapsedTime: ElapsedTime = ElapsedTime()
    private var xPID = PIDController(DriveConstants.PID_X)
    private var yPID = PIDController(DriveConstants.PID_Y)

    var path: Path? = null
        set(value) {
            field = value
            // Reset the follower
            reset()
        }

    var targetState: MotionState? = null
        get() {
            if (motionProfile == null) {
                FileLogger.error("Follower", "Motion profile is not set")
                return null
            }
            // Get the current time in seconds since the follower started
            val t = elapsedTime.seconds()
            return motionProfile!![t]
        }

    fun update() {
        if (motionProfile == null || path == null) {
            FileLogger.error("Follower", "No path or motion profile set")
            return
        }
        // Get the time since the follower started
        val t = elapsedTime.seconds()
        // Get the target state from the motion profile
        val targetState = motionProfile!![t]
        // Calculate the target point based on distance along the path
        val pathT = targetState.x / path!!.getLength()
        val targetPoint = path!!.getPoint(pathT)
        val targetPointFirstDerivative = path!!.getTangent(pathT).normalize()
        val targetPointSecondDerivative = path!!.getSecondDerivative(pathT)

        // Calculate the position error and convert to robot-centric coordinates
        val positionError = targetPoint - Bot.localizer.pose
        positionError.rotate(-Bot.localizer.pose.heading)

        // TODO: Heading interpolation

        // Calculate the PID outputs
        var xPower = xPID.update(positionError.x, Bot.dt)
        var yPower = yPID.update(positionError.y, Bot.dt)

        // Calculate 2D target velocity and acceleration based on path derivatives
        val targetVelocity = targetPointFirstDerivative * targetState.v
        val targetAcceleration = targetPointSecondDerivative * (targetState.v * targetState.v) +
                targetPointFirstDerivative * targetState.a

        // Convert target velocity and acceleration to robot-centric coordinates
        targetVelocity.rotate(-Bot.localizer.pose.heading)
        targetAcceleration.rotate(-Bot.localizer.pose.heading)

        // Add feedforward terms
        xPower += (targetVelocity.x * DriveConstants.KV) +
                (targetAcceleration.x * DriveConstants.KA) +
                (sign(targetVelocity.x) * DriveConstants.KS)

        yPower += (targetVelocity.y * DriveConstants.KV) +
                (targetAcceleration.y * DriveConstants.KA) +
                (sign(targetVelocity.y) * DriveConstants.KS)

        // Drive based on the calculated powers
        Bot.mecanumBase.moveVector(xPower, yPower, 0.0)
    }

    private fun reset() {
        // Recalculate the motion profile when the path is set
        calculateMotionProfile()
        // Reset the elapsed time
        elapsedTime.reset()
    }

    private fun calculateMotionProfile() {
        if (path != null) {
            val totalDistance = path!!.getLength()
            val startState = MotionState(0.0, 0.0, 0.0)
            val endState = MotionState(totalDistance, 0.0, 0.0)
            val velocityConstraint = { s: Double ->
                // Velocity constraint based on path curvature
                val t = s / totalDistance
                val k = path!!.getCurvature(t)
                val curveMaxVelocity = sqrt(DriveConstants.MAX_CENTRIPETAL_ACCELERATION / abs(k))
                if (curveMaxVelocity.isNaN()) {
                    DriveConstants.MAX_DRIVE_VELOCITY
                } else {
                    min(DriveConstants.MAX_DRIVE_VELOCITY, curveMaxVelocity)
                }
            }
            val accelerationConstraint = { s: Double ->
                // Constant acceleration constraint
                DriveConstants.MAX_DRIVE_ACCELERATION
            }
            motionProfile = MotionProfileGenerator.generateMotionProfile(
                startState,
                endState,
                velocityConstraint,
                accelerationConstraint,
            )
        } else {
            motionProfile = null
        }
    }
}
