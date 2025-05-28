package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.MecanumBase
import org.firstinspires.ftc.teamcode.helpers.TelemetryInterface
import org.firstinspires.ftc.teamcode.localization.Localizer
import org.firstinspires.ftc.teamcode.localization.localizers.Pinpoint

class Bot () {
    companion object {
        lateinit var mecanumBase: MecanumBase
            private set // Prevent external modification

        lateinit var telemetry: TelemetryInterface
            private set

        lateinit var localizer: Localizer

        fun initialize(hardwareMap: HardwareMap) {
            mecanumBase = MecanumBase(hardwareMap)
            localizer = Pinpoint(hardwareMap)
        }

        fun setTelemetry(telemetry: TelemetryInterface) {
            this.telemetry = telemetry
        }
    }
}
