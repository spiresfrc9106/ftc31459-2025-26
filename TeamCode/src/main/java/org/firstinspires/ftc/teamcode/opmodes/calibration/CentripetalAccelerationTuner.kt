package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.DashboardPlotter
import org.firstinspires.ftc.teamcode.pathing.paths.HermitePath
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import org.firstinspires.ftc.teamcode.pathing.motionprofiles.ConstantMotionProfile

@Autonomous(name = "Centripetal Acceleration Tuner", group = "Calibration")
class CentripetalAccelerationTuner : OpMode() {
    // Max centripetal acceleration = v^2 / r
    // Find the highest velocity where the robot follows the path

    // @Config
    class CentripetalAccelerationConstants{
        companion object {
            @JvmField var TARGET_VELOCITY = 50.0 // Target velocity in cm/s
            @JvmField var RADIUS = 25.0 // Radius of the circular path in cm
        }
    }

    override fun init() {
        val RADIUS = CentripetalAccelerationConstants.RADIUS
        val TARGET_VELOCITY = CentripetalAccelerationConstants.TARGET_VELOCITY

        Bot.initialize(hardwareMap, telemetry, Pose(RADIUS, 0.0))

        val k = 0.55228475 * 3.0 * RADIUS // Constant for creating an arc with Hermite curves
        Bot.follower.path = HermitePath.Builder()
            .addPoint(Pose(RADIUS, 0.0), Pose(0.0, k))
            .addPoint(Pose(0.0, RADIUS), Pose(-k, 0.0))
            .addPoint(Pose(-RADIUS, 0.0), Pose(0.0, -k))
            .addPoint(Pose(0.0, -RADIUS), Pose(k, 0.0))
            .addPoint(Pose(RADIUS, 0.0), Pose(0.0, k))
            .build()

        Bot.follower.motionProfile = ConstantMotionProfile(TARGET_VELOCITY)
    }

    override fun loop() {
        // Update bot
        Bot.update()

        // Add telemetry data
        Bot.telemetryPacket.put("Target Velocity", CentripetalAccelerationConstants.TARGET_VELOCITY)
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity)

        // Plot data on field view
        DashboardPlotter.plotBotPosition(Bot.telemetryPacket, Bot.localizer.pose)
        DashboardPlotter.plotPath(Bot.telemetryPacket, Bot.follower.path!!)
        DashboardPlotter.plotCircle(Bot.telemetryPacket, Bot.localizer.pose, DriveConstants.LOOK_AHEAD_DISTANCE)
        DashboardPlotter.plotPoint(Bot.telemetryPacket, Bot.follower.lookaheadPoint)
        Bot.sendTelemetryPacket()
    }

    override fun stop() {
        Bot.stop()
    }
}