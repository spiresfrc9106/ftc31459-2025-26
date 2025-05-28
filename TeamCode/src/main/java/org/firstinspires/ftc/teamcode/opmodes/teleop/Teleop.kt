package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface
import org.firstinspires.ftc.teamcode.opmodes.teleop.drivers.*

@TeleOp(name = "Teleop", group = "Teleop")
class Teleop : OpMode() {
    private lateinit var driver1: TeleopDriver1
    private lateinit var driver2: TeleopDriver2

    private val timer: ElapsedTime = ElapsedTime()

    private var dt: Double = 0.0 // Delta time in milliseconds
    private var prevTime = timer.milliseconds()

    override fun init() {
        Bot.initialize(hardwareMap)
        Bot.setTelemetry(TelemetryInterface(telemetry, FtcDashboard.getInstance().telemetry))
        driver1 = TeleopDriver1(gamepad1)
        driver2 = TeleopDriver2(gamepad2)
        timer.reset()
    }

    override fun loop() {
        // Update localizer
        Bot.localizer.update(dt / 1000.0) // Convert milliseconds to seconds

        // Update gamepad inputs
        driver1.update()
        driver2.update()

        // Update telemetry
        updateTelemetry()

        // Update delta time
        dt = timer.milliseconds() - prevTime
        prevTime = timer.milliseconds()
    }

    private fun updateTelemetry() {
        Bot.telemetry.addData("Drive Speed", driver1.driveSpeed)
        Bot.telemetry.addData("Pose", Bot.localizer.pose)
        Bot.telemetry.addData("Velocity", Bot.localizer.velocity)
        Bot.telemetry.update()
    }
}
