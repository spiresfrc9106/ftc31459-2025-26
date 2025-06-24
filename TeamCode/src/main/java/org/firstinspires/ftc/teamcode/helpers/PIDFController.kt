package org.firstinspires.ftc.teamcode.helpers

class PIDFController(
    val Kp: Double,
    val Ki: Double,
    val Kd: Double,
    val Kf: Double
) {
    private var integral = 0.0
    private var lastError = 0.0

    fun update(target: Double, current: Double, dt: Double): Double {
        val error = target - current
        integral += error * dt
        val derivative = (error - lastError) / dt
        lastError = error

        return (Kf * target) + (Kp * error) + (Ki * integral) + (Kd * derivative)
    }

    fun reset() {
        integral = 0.0
        lastError = 0.0
    }
}
