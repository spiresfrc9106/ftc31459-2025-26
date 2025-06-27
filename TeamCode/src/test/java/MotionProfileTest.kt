import org.firstinspires.ftc.teamcode.pathing.motionprofiles.TrapezoidalMotionProfile
import org.junit.Test

class MotionProfileTest {
    @Test
    fun testGetVelocity() {
        // Create a mock implementation of MotionProfile for testing
        val pathLength = 5.0
        val motionProfile = TrapezoidalMotionProfile(pathLength)

        for (i in 0..100) {
            val t = i / 100.0 * pathLength // Scale t to the path length
            val velocity = motionProfile.getVelocity(t)
            println("($t, $velocity)")
        }
    }
}
