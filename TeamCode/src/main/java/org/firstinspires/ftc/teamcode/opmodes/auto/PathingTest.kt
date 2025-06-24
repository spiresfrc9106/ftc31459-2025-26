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

@Autonomous(name = "Pathing Test", group = "Testing")
class PathingTest : OpMode() {
    val dashboard = FtcDashboard.getInstance()

    override fun init() {
        Bot.initialize(hardwareMap)
        Bot.setTelemetry(TelemetryInterface(telemetry, FtcDashboard.getInstance().telemetry))

        // Set target path for the follower
        Bot.follower.path = CompoundPath.PolyLineBuilder()
            .addPoint(Pose(0.0, 0.0, 0.0))
            .addPoint(Pose(100.0, 0.0, 0.0))
            .addPoint(Pose(200.0, 100.0, 0.0))
            .addPoint(Pose(200.0, 200.0, 0.0))
            .build()
    }

    override fun loop() {
        // Update bot
        Bot.update()

        // Check if the target is reached
        if (Bot.follower.reachedTarget()) {
            requestOpModeStop()
        }

        // Plot data on the dashboard
        var packet = TelemetryPacket(false)
        DashboardPlotter.plotBotPosition(packet, Bot.localizer.pose)
        DashboardPlotter.plotPath(packet, Bot.follower.path!!)
        DashboardPlotter.plotPoint(packet, Bot.follower.lookaheadPoint)
        DashboardPlotter.plotCircle(packet, Bot.localizer.pose, DriveConstants.LOOK_AHEAD_DISTANCE)
        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        Bot.stop()
    }
}
