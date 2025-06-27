package org.firstinspires.ftc.teamcode.pathing.motionprofiles

interface MotionProfile {
    /**
     * Gets the velocity at the given distance traveled.
     * @param s The distance already traveled.
     * @return The velocity at the given distance traveled.
     */
    fun getVelocity(s: Double): Double
}
