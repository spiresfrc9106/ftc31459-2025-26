package org.firstinspires.ftc.teamcode.pathing.motionprofilers

interface MotionProfile {
    /**
     * Gets the velocity at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The velocity at the given parameter
     */
    fun getVelocity(t: Double): Double

    /**
     * Gets the acceleration at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The acceleration at the given parameter
     */
    fun getAcceleration(t: Double): Double

    // TODO: Not finished
}