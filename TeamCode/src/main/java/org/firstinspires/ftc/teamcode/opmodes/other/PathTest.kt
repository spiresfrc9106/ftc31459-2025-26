package org.firstinspires.ftc.teamcode.opmodes.other

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.DashboardPlotter
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.follower.DriveConstants
import org.firstinspires.ftc.teamcode.pathing.paths.HermitePath

@TeleOp(name = "Path Test", group = "Testing")
class PathTest : OpMode() {
    override fun init() {
        Bot.initialize(hardwareMap, telemetry)
        Bot.follower.path = HermitePath(
            startPose = Pose(0.0, 0.0, 0.0),
            endPose = Pose(100.0, 100.0, 0.0),
            startVelocity = Pose(100.0, 0.0),
            endVelocity = Pose(100.0, 0.0)
        )
        Bot.telemetryPacket.put("t=0.0", Bot.follower.path!!.getPoint(0.0))
        Bot.telemetryPacket.put("t=0.25", Bot.follower.path!!.getPoint(0.25))
        Bot.telemetryPacket.put("t=0.5", Bot.follower.path!!.getPoint(0.5))
        Bot.telemetryPacket.put("t=0.75", Bot.follower.path!!.getPoint(0.75))
        Bot.telemetryPacket.put("t=1.0", Bot.follower.path!!.getPoint(1.0))
        Bot.sendTelemetryPacket()
    }
    override fun loop() {
        Bot.update()

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
