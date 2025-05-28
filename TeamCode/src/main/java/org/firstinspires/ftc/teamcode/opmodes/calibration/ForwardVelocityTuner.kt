package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Bot

class ForwardVelocityTuner : OpMode() {
    private val timer: ElapsedTime = ElapsedTime()

    private var dt: Double = 0.0 // Delta time in milliseconds
    private var prevTime = timer.milliseconds()

    private var maxVelocity = 0.0 // Maximum velocity in cm/s

    override fun init() {
        Bot.initialize(hardwareMap)
        Bot.localizer.reset() // Reset the localizer to the origin
        timer.reset()
    }

    override fun loop() {
        // Update localizer
        Bot.localizer.update(dt / 1000.0) // Convert milliseconds to seconds

        // Drive right 100 cm
        if (Bot.localizer.pose.y < 100) {
            Bot.mecanumBase.moveVector(0.0, 1.0, 0.0, adjustForStrafe = false)
            // Update max velocity
            if (Bot.localizer.velocity.y > maxVelocity) {
                maxVelocity = Bot.localizer.velocity.y
            }
        } else {
            Bot.mecanumBase.stop()
        }
        
        // Update telemetry
        telemetry.addData("Max Velocity (cm/s)", maxVelocity)
        telemetry.addData("Current Pose", Bot.localizer.pose)
        telemetry.addData("Current Velocity", Bot.localizer.velocity)
        telemetry.update()
        
        // Update delta time
        dt = timer.milliseconds() - prevTime
        prevTime = timer.milliseconds()
    }
}
