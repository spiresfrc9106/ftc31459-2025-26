package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.LinearPath
import kotlin.math.PI

/**
 * Test to be run before matches to ensure all hardware works
 * as intended and to rule out many possible software issues.
 * If software issues arise, please run diff
 */
@Autonomous(name = "Pre Flight Check", group = "Calibration")
class PreFlightCheck : OpMode() {

    enum class Test {
        DRIVE_F,
        DRIVE_B,
        DRIVE_R,
        DRIVE_L,
        TURN_R,
        TURN_L,
        STOP
    }

    private var currentTest = Test.DRIVE_F
    override fun init() {
        Bot.initialize(hardwareMap, telemetry)

        Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(0.0, 50.0)) // Path for DRIVE_F

        telemetry.addData("Current Test", currentTest)
    }

    override fun loop() {
        Bot.update()
        when (currentTest) {
            Test.DRIVE_F -> driveF()
            Test.DRIVE_B -> driveB()
            Test.DRIVE_R -> driveR()
            Test.DRIVE_L -> driveL()
            Test.TURN_R -> turnR()
            Test.TURN_L -> turnL()
            Test.STOP -> stop()
        }
        telemetry.addData("Current Test", currentTest)
    }

    override fun stop() {
        Bot.stop()
        requestOpModeStop()
    }


    private fun driveF() {
        if (Bot.follower.reachedTarget()) {
            Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(0.0, 0.0)) // Path for DRIVE_B
            currentTest = Test.DRIVE_B
        }
    }

    private fun driveB() {
        if (Bot.follower.reachedTarget()) {
            Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(50.0, 0.0)) // Path for DRIVE_R
            currentTest = Test.DRIVE_R
        }
    }

    private fun driveR() {
        if (Bot.follower.reachedTarget()) {
            Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(0.0, 0.0)) // Path for DRIVE_L
            currentTest = Test.DRIVE_L
        }
    }

    private fun driveL() {
        if (Bot.follower.reachedTarget()) {
            Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(0.0, 0.0, -PI+0.1)) // Path for TURN R
            currentTest = Test.TURN_R
        }
    }

    private fun turnR() {
        if (Bot.follower.reachedTarget()) {
            Bot.follower.path = LinearPath(Bot.localizer.pose, Pose(0.0, 0.0, 0.0)) // Path for TURN L
            currentTest = Test.TURN_L
        }
    }

    private fun turnL() {
        if (Bot.follower.reachedTarget()) {
            currentTest = Test.STOP
        }
    }
}