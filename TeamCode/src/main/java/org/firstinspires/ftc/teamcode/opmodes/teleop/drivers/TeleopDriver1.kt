package org.firstinspires.ftc.teamcode.opmodes.teleop.drivers

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.Toggle

class TeleopDriver1 (var gamepad: Gamepad) {
    var driveSpeed = 0.5

    // Toggles
    private var incDriveSpeed: Toggle = Toggle(false)
    private var decDriveSpeed: Toggle = Toggle(false)

    fun update() {
        drive()
        updateDriveSpeed()
    }

    fun drive() {
        Bot.mecanumBase.moveVector(
            gamepad.left_stick_x.toDouble(),
            -gamepad.left_stick_y.toDouble(),
            gamepad.right_stick_x.toDouble(),
            driveSpeed
        )
    }

    fun updateDriveSpeed() {
        incDriveSpeed.toggle(gamepad.right_bumper)
        decDriveSpeed.toggle(gamepad.left_bumper)
        if (incDriveSpeed.justChanged) {
            driveSpeed += 0.1
        }
        if (decDriveSpeed.justChanged) {
            driveSpeed -= 0.1
        }
        driveSpeed = driveSpeed.coerceIn(0.1, 1.0)
    }
}
