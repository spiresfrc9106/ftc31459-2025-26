package org.firstinspires.ftc.teamcode.pathing.follower

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger
import org.firstinspires.ftc.teamcode.helpers.PIDController
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionProfile
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionProfileGenerator
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionState
import org.firstinspires.ftc.teamcode.pathing.paths.Path
import kotlin.math.*

class Follower {
    var motionProfile: MotionProfile? = null
    private var elapsedTime: ElapsedTime = ElapsedTime()
    private var xPID = PIDController(FollowerConstants.PID_X)
    private var yPID = PIDController(FollowerConstants.PID_Y)

    var path: Path? = null
        set(value) {
            field = value
            // Reset the follower
            reset()
        }

    var done: Boolean = false
        get() {
            if (motionProfile == null || path == null) {
                FileLogger.error("Follower", "No path or motion profile set")
                return false
            }
            return elapsedTime.seconds() > motionProfile!!.duration()
            // Maybe add position or velocity tolerance
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
        val t = elapsedTime.seconds().coerceAtMost(motionProfile!!.duration())
        // Get the target state from the motion profile
        val targetState = motionProfile!![t]

        // Calculate the parameter t for the path based on the target state
        val pathT = path!!.getTFromLength(targetState.x)

        // Get the target point, first derivative (tangent), and second derivative (acceleration) from the path
        val targetPoint = path!!.getPoint(pathT)
        val targetPointFirstDerivative = path!!.getTangent(pathT).normalize()
        val targetPointSecondDerivative = path!!.getSecondDerivative(pathT)

        // Calculate the position error and convert to robot-centric coordinates
        val positionError = targetPoint - Bot.localizer.pose
        positionError.rotate(-Bot.localizer.pose.heading)

        // Calculate 2D target velocity and acceleration based on path derivatives
        val targetVelocity = targetPointFirstDerivative * targetState.v
        val targetAcceleration = targetPointSecondDerivative * (targetState.v * targetState.v) +
                targetPointFirstDerivative * targetState.a

        // Convert target velocity and acceleration to robot-centric coordinates
        targetVelocity.rotate(-Bot.localizer.pose.heading)
        targetAcceleration.rotate(-Bot.localizer.pose.heading)

        // Velocity error for PID
        val velocityError = targetVelocity - Bot.localizer.velocity

        // TODO: Heading interpolation

        // Calculate the PID outputs
        var xCorrection = xPID.update(positionError.x, Bot.dt, velocityError.x)
        var yCorrection = yPID.update(positionError.y, Bot.dt, velocityError.y)

//        // Add feedforward terms
//        xCorrection += (targetVelocity.x * FollowerConstants.KV[0]) +
//                (targetAcceleration.x * FollowerConstants.KA[0]) +
//                if (abs(targetVelocity.x) > 1e-3) FollowerConstants.KS[0] * sign(targetVelocity.x) else 0.0
//                // Only add kS if the target velocity is significant
//
//        yCorrection += (targetVelocity.y * FollowerConstants.KV[1]) +
//                (targetAcceleration.y * FollowerConstants.KA[1]) +
//                if (abs(targetVelocity.y) > 1e-3) FollowerConstants.KS[1] * sign(targetVelocity.y) else 0.0

        //xCorrection = xCorrection.coerceIn(-1.0, 1.0)
        //yCorrection = yCorrection.coerceIn(-1.0, 1.0)

        // Calculate adjusted velocity based on PID corrections
        // TODO: Add heading correction
        val adjustedVelocity = targetVelocity + Pose(xCorrection, yCorrection, 0.0)
        Bot.mecanumBase.setDriveVA(adjustedVelocity, targetAcceleration)
    }

    fun start() {
        // Reset the elapsed time
        elapsedTime.reset()
        // Reset the PID controllers
        xPID.reset()
        yPID.reset()
    }

    private fun reset() {
        // Recalculate the motion profile when the path is set
        calculateMotionProfile()
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
                val curveMaxVelocity = sqrt(FollowerConstants.MAX_CENTRIPETAL_ACCELERATION / abs(k))
                if (curveMaxVelocity.isNaN()) {
                    FollowerConstants.MAX_DRIVE_VELOCITY
                } else {
                    min(FollowerConstants.MAX_DRIVE_VELOCITY, curveMaxVelocity)
                }
            }
            val accelerationConstraint = { s: Double ->
                // Constant acceleration constraint
                FollowerConstants.MAX_DRIVE_ACCELERATION
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
