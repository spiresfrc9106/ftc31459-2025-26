package org.firstinspires.ftc.teamcode.opmodes.calibration

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime

import org.firstinspires.ftc.teamcode.Bot
import org.firstinspires.ftc.teamcode.helpers.FileLogger

@Autonomous
class FullMotionCalibration : OpMode() {
    // Information determined during calibration
    private var maxVelocityF = 0.0
    private var maxVelocityB = 0.0
    private var maxVelocityR = 0.0
    private var maxVelocityL = 0.0
    private var maxAccelF = 0.0
    private var maxAccelB = 0.0
    private var maxAccelR = 0.0
    private var maxAccelL = 0.0

    private val accelTolerance = 1
    private val holdTime = 0.6
    private val power = 0.7
    private var reachedAccel = false

    private enum class Test {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        STOP
    }

    private enum class TestStage {
        ACCEL,
        HOLD,
        DECEL
    }

    private var currentTest = Test.FORWARD
    private var testStage = TestStage.ACCEL
    private var timer = ElapsedTime()

    override fun init() {
        Bot.initialize(hardwareMap, telemetry)
        Bot.localizer.reset() // Reset the localizer to the origin

        Bot.telemetryPacket.put("Test", currentTest.toString())
        Bot.telemetryPacket.put("Stage", testStage.toString())
        Bot.telemetryPacket.put("Reached Accel", reachedAccel.toString())
        Bot.telemetryPacket.put("AccelY", Bot.localizer.acceleration.y)
        Bot.telemetryPacket.put("VelY", Bot.localizer.velocity.y)
        Bot.telemetryPacket.put("AccelX", Bot.localizer.acceleration.x)
        Bot.telemetryPacket.put("VelX", Bot.localizer.velocity.x)
        Bot.telemetryPacket.put("dt", Bot.dt.toString())
        Bot.sendTelemetryPacket()
    }

    override fun loop() {
        Bot.update()
        when (currentTest) {
            Test.FORWARD -> {
                forwardTest()
            }
            Test.BACKWARD -> {
                backwardTest()
            }
            Test.LEFT -> {
                leftTest()
            }
            Test.RIGHT -> {
                rightTest()
            }
            Test.STOP -> requestOpModeStop()
        }
        Bot.telemetryPacket.put("Test", currentTest.toString())
        Bot.telemetryPacket.put("Stage", testStage.toString())
        Bot.telemetryPacket.put("Reached Accel", reachedAccel.toString())
        Bot.telemetryPacket.put("AccelY", Bot.localizer.acceleration.y)
        Bot.telemetryPacket.put("VelY", Bot.localizer.velocity.y)
        Bot.telemetryPacket.put("AccelX", Bot.localizer.acceleration.x)
        Bot.telemetryPacket.put("VelX", Bot.localizer.velocity.x)
        Bot.telemetryPacket.put("dt", Bot.dt.toString())
        Bot.sendTelemetryPacket()
    }

    override fun stop() {
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxVelocityF", maxVelocityF.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxVelocityB", maxVelocityB.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxVelocityR", maxVelocityR.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxVelocityL", maxVelocityL.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxAccelF", maxAccelF.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxAccelB", maxAccelB.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxAccelR", maxAccelR.toString())
        FileLogger.log(FileLogger.Companion.LogLevel.INFO, "maxAccelL", maxAccelL.toString())

        Bot.stop()
    }

    private fun forwardTest() {
        val velocity = Bot.localizer.velocity.y
        val accel = Bot.localizer.acceleration.y
        when (testStage) {
            TestStage.ACCEL -> {
                Bot.mecanumBase.moveVector(0.0, 1.0, 0.0, power, false)

                if (accel > maxAccelF) { maxAccelF = accel } // Update max accel
                if (accel > accelTolerance) { reachedAccel = true }
                if (accel < accelTolerance && reachedAccel) { // Max velocity reached
                    maxVelocityF = velocity
                    testStage = TestStage.HOLD
                    timer.reset()
                }
            }
            TestStage.HOLD -> {
                Bot.mecanumBase.moveVector(0.0, 1.0, 0.0, power, false)

                if (velocity > maxVelocityF) { maxVelocityF = velocity }

                if (timer.seconds() > holdTime) {
                    testStage = TestStage.DECEL
                    timer.reset()
                }
            }
            TestStage.DECEL -> {
                Bot.mecanumBase.stop()
                if (Bot.localizer.velocity.getLength() < 0.1) {
                    testStage = TestStage.ACCEL
                    currentTest = Test.BACKWARD
                    reachedAccel = false
                }
            }
        }

    }
    private fun backwardTest() {
        val velocity = -Bot.localizer.velocity.y
        val accel = -Bot.localizer.acceleration.y
        when (testStage) {
            TestStage.ACCEL -> {
                Bot.mecanumBase.moveVector(0.0, -1.0, 0.0, power, false)

                if (accel > maxAccelB) { maxAccelB = accel } // Update max accel
                if (accel > accelTolerance) { reachedAccel = true }
                if (accel < accelTolerance && reachedAccel) { // Max velocity reached
                    maxVelocityB = velocity
                    testStage = TestStage.HOLD
                    timer.reset()
                }
            }
            TestStage.HOLD -> {
                Bot.mecanumBase.moveVector(0.0, -1.0, 0.0, power, false)

                if (velocity > maxVelocityB) { maxVelocityB = velocity }

                if (timer.seconds() > holdTime) {
                    testStage = TestStage.DECEL
                    timer.reset()
                }
            }
            TestStage.DECEL -> {
                Bot.mecanumBase.stop()
                if (Bot.localizer.velocity.getLength() < 0.1) {
                    testStage = TestStage.ACCEL
                    currentTest = Test.LEFT
                    reachedAccel = false
                }
            }
        }
    }
    private fun leftTest() {
        val velocity = -Bot.localizer.velocity.x
        val accel = -Bot.localizer.acceleration.x
        when (testStage) {
            TestStage.ACCEL -> {
                Bot.mecanumBase.moveVector(-1.0, 0.0, 0.0, power, false)

                if (accel > maxAccelL) { maxAccelL = accel } // Update max accel
                if (accel > accelTolerance) { reachedAccel = true }
                if (accel < accelTolerance && reachedAccel) { // Max velocity reached
                    maxVelocityL = velocity
                    testStage = TestStage.HOLD
                    timer.reset()
                }
            }
            TestStage.HOLD -> {
                Bot.mecanumBase.moveVector(-1.0, 0.0, 0.0, power, false)

                if (velocity > maxVelocityL) { maxVelocityL = velocity }

                if (timer.seconds() > holdTime) {
                    testStage = TestStage.DECEL
                    timer.reset()
                }
            }
            TestStage.DECEL -> {
                Bot.mecanumBase.stop()
                if (Bot.localizer.velocity.getLength() < 0.1) {
                    testStage = TestStage.ACCEL
                    currentTest = Test.RIGHT
                    reachedAccel = false
                }
            }
        }
    }
    private fun rightTest() {
        val velocity = Bot.localizer.velocity.x
        val accel = Bot.localizer.acceleration.x
        when (testStage) {
            TestStage.ACCEL -> {
                Bot.mecanumBase.moveVector(1.0, 0.0, 0.0, power, false)

                if (accel > maxAccelR) { maxAccelR = accel } // Update max accel
                if (accel > accelTolerance) { reachedAccel = true }
                if (accel < accelTolerance && reachedAccel) { // Max velocity reached
                    maxVelocityR = velocity
                    testStage = TestStage.HOLD
                    timer.reset()
                }
            }
            TestStage.HOLD -> {
                Bot.mecanumBase.moveVector(1.0, 0.0, 0.0, power, false)

                if (velocity > maxVelocityR) { maxVelocityR = velocity }

                if (timer.seconds() > holdTime) {
                    testStage = TestStage.DECEL
                    timer.reset()
                }
            }
            TestStage.DECEL -> {
                Bot.mecanumBase.stop()
                if (Bot.localizer.velocity.getLength() < 0.1) {
                    testStage = TestStage.ACCEL
                    currentTest = Test.STOP
                    reachedAccel = false
                }
            }
        }
    }
}