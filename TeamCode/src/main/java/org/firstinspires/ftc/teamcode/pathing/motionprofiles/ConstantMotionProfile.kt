package org.firstinspires.ftc.teamcode.pathing.motionprofiles

class ConstantMotionProfile(val velocity: Double) : MotionProfile{
    override fun getVelocity(s: Double): Double {
        return velocity
    }
}
