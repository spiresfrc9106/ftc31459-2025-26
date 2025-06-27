package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.DashboardPlotter
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.CompoundPath
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import org.firstinspires.ftc.teamcode.pathing.paths.HermitePath
import org.firstinspires.ftc.teamcode.pathing.paths.LinearPath
import kotlin.math.PI

@Autonomous(name = "Pathing Test", group = "Testing")
class PathingTest : OpMode() {
    val dashboard = FtcDashboard.getInstance()

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)

        // Set target path for the follower
        Bot.follower.path = HermitePath.Builder()
            .addPoint(Pose(0.0,0.0))
            .addPoint(Pose(50.0,0.0))
            .addPoint(Pose(50.0,50.0))
            .addPoint(Pose(0.0,50.0))
            .addPoint(Pose(0.0,100.0))
            .addPoint(Pose(50.0,100.0))
            .build()

        Bot.telemetryPacket.put("Target Heading", Bot.follower.lookaheadPoint.heading)
        Bot.telemetryPacket.put("Expected Heading", Bot.follower.path!!.getClosestPoint(Bot.localizer.pose).heading)
        Bot.telemetryPacket.put("Current Heading", Bot.localizer.pose.heading)
        Bot.telemetryPacket.put("Target Velocity", Bot.follower.getTargetSpeed())
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
        Bot.telemetryPacket.put("Target Velocity", Bot.follower.getTargetSpeed())
        Bot.telemetryPacket.put("Current Velocity", Bot.localizer.velocity.getLength())

        // Field view
        DashboardPlotter.plotGrid(Bot.telemetryPacket)
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
