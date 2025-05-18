package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.hardware.MecanumBase
import org.firstinspires.ftc.teamcode.localization.Localizer
import org.firstinspires.ftc.teamcode.localization.localizers.ThreeWheelOdometry

class Bot () {
    companion object {
        lateinit var mecanumBase: MecanumBase
            private set // Prevent external modification

        lateinit var localizer: Localizer
            private set

        fun initialize(hardwareMap: HardwareMap) {
            mecanumBase = MecanumBase(hardwareMap)
            localizer = ThreeWheelOdometry(hardwareMap)
        }
    }
}
