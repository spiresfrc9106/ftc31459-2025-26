package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.MecanumBase
import org.firstinspires.ftc.teamcode.helpers.FileLogger
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface
import org.firstinspires.ftc.teamcode.localization.Localizer
import org.firstinspires.ftc.teamcode.localization.localizers.Pinpoint
import org.firstinspires.ftc.teamcode.pathing.follower.Follower

class Bot () {
    companion object {
        private val timer: ElapsedTime = ElapsedTime()
        private var prevTime: Double = timer.milliseconds()

        var dt: Double = 0.0 // Delta time in milliseconds
            private set // Prevent external modification

        lateinit var mecanumBase: MecanumBase
            private set

        lateinit var telemetry: TelemetryInterface
            private set

        lateinit var follower: Follower
            private set

        lateinit var localizer: Localizer

        fun initialize(hardwareMap: HardwareMap) {
            mecanumBase = MecanumBase(hardwareMap)
            localizer = Pinpoint(hardwareMap)
            follower = Follower()
        }

        fun update() {
            // Update localizer
            localizer.update(dt / 1000.0) // Convert milliseconds to seconds

            // Update follower
            follower.update()

            // Update delta time
            dt = timer.milliseconds() - prevTime
            prevTime = timer.milliseconds()
        }

        fun setTelemetry(telemetry: TelemetryInterface) {
            this.telemetry = telemetry
        }

        fun stop() {
            mecanumBase.stop()
            follower.path = null // Clear the path
            FileLogger.flush() // Save any pending logs
        }
    }
}
