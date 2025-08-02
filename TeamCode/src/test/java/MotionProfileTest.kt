import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionProfileGenerator
import org.firstinspires.ftc.teamcode.pathing.motionprofile.MotionState
import org.junit.Test
import kotlin.math.min

class MotionProfileTest {
    @Test
    fun test() {
        val motionProfile = MotionProfileGenerator.generateMotionProfile(
            MotionState(0.0, 0.0, 0.0),
            MotionState(50.0, 0.0, 0.0),
            { s -> min(5.0, (s-25.0)*(s-25.0) / 10.0 + 1.0) },
            { 1.0 },
            0.1
        )
        val startState = motionProfile.start()
        val endState = motionProfile.end()
        val duration = motionProfile.duration()
        println("Start State: $startState")
        println("End State: $endState")
        println("Duration: $duration")
        for (i in 0..100) {
            val t = i / 100.0 * duration
            val state = motionProfile[t]
            println("$t: $state")
        }
    }

    @Test
    fun tempTest() {
        val motionProfile = MotionProfileGenerator.generateMotionProfile(
            MotionState(0.0, 0.0, 0.0),
            MotionState(50.0, 0.0, 0.0),
            { s -> min(5.0, (s-25.0)*(s-25.0) / 10.0 + 1.0) },
            { 1.0 }
        )
    }
}
