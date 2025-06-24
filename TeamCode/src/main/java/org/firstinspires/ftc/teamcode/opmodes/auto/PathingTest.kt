package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.DashboardPlotter
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.CompoundPath
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import org.firstinspires.ftc.teamcode.pathing.motionprofiles.TrapezoidalMotionProfile
import kotlin.math.PI

@Autonomous(name = "Pathing Test", group = "Testing")
class PathingTest : OpMode() {
    val dashboard = FtcDashboard.getInstance()

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)

        // Set target path for the follower
        Bot.follower.path = CompoundPath.PolyLineBuilder()
            .addPoint(Pose(0.0, 0.0, 0.0))
            .addPoint(Pose(100.0, 0.0, -PI / 2))
            .addPoint(Pose(200.0, 100.0, -PI / 2))
            .addPoint(Pose(200.0, 200.0, PI))
            .build()

        Bot.telemetryPacket.put("Target Heading", Bot.follower.lookaheadPoint.heading)
        Bot.telemetryPacket.put("Expected Heading", Bot.follower.path!!.getClosestPoint(Bot.localizer.pose).heading)
        Bot.telemetryPacket.put("Current Heading", Bot.localizer.pose.heading)
        Bot.telemetryPacket.put("Target Velocity", Bot.follower.getTargetSpeed(Bot.follower.lookaheadPointT))
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.getLength())
        Bot.sendTelemetryPacket()
    }

    override fun loop() {
        // Update bot
        Bot.update()

        // Check if the target is reached
        if (Bot.follower.reachedTarget()) {
            requestOpModeStop()
        }

        // Plot data on the dashboard

        // Graph view
        Bot.telemetryPacket.put("Target Heading", Bot.follower.lookaheadPoint.heading)
        Bot.telemetryPacket.put("Expected Heading", Bot.follower.path!!.getClosestPoint(Bot.localizer.pose).heading)
        Bot.telemetryPacket.put("Current Heading", Bot.localizer.pose.heading)
        Bot.telemetryPacket.put("Target Velocity", Bot.follower.getTargetSpeed(Bot.follower.lookaheadPointT))
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.getLength())

        // Field view
        DashboardPlotter.plotBotPosition(Bot.telemetryPacket, Bot.localizer.pose)
        DashboardPlotter.plotPath(Bot.telemetryPacket, Bot.follower.path!!)
        DashboardPlotter.plotPoint(Bot.telemetryPacket, Bot.follower.lookaheadPoint)
        DashboardPlotter.plotPoint(Bot.telemetryPacket, Bot.follower.path!!.getClosestPoint(Bot.localizer.pose))
        DashboardPlotter.plotCircle(Bot.telemetryPacket, Bot.localizer.pose, DriveConstants.LOOK_AHEAD_DISTANCE)
        Bot.sendTelemetryPacket()
    }

    override fun stop() {
        Bot.stop()
    }
}
