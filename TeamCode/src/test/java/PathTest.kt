import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.HermitePath
import org.firstinspires.ftc.teamcode.pathing.paths.LinearPath
import org.junit.Test
import kotlin.math.sqrt

class PathTest {
    @Test
    fun linearPath() {
        val path = LinearPath.Builder()
            .addPoint(Pose(0.0, 0.0))
            .addPoint(Pose(1.0, 1.0))
            .build()
        assert(path.getPoint(0.0) == Pose(0.0, 0.0))
        assert(path.getPoint(0.5) == Pose(0.5, 0.5))
        assert(path.getPoint(1.0) == Pose(1.0, 1.0))
        assert(path.getLength() == sqrt(2.0))
        assert(path.getLookaheadPoint(Pose(0.0, 0.0), 1.0) == Pose(sqrt(2.0)/2.0, sqrt(2.0)/2.0))
    }

    @Test
    fun linearPath2() {
        val path = LinearPath.Builder()
            .addPoint(Pose(1.0, 1.0))
            .addPoint(Pose(2.0, 2.0))
            .build()
        assert(path.getPoint(0.0) == Pose(1.0, 1.0))
        assert(path.getPoint(0.5) == Pose(1.5, 1.5))
        assert(path.getPoint(1.0) == Pose(2.0, 2.0))
        assert(path.getLength() == sqrt(2.0))
        assert(path.getLookaheadPoint(Pose(0.0, 0.0), 1.0) == null)
    }

    @Test
    fun compoundLinearPath() {
        val path = LinearPath.Builder()
            .addPoint(Pose(0.0, 0.0))
            .addPoint(Pose(1.0, 1.0))
            .addPoint(Pose(2.0, 2.0))
            .build()
        assert(path.getPoint(0.0) == Pose(0.0, 0.0))
        assert(path.getPoint(0.5) == Pose(1.0, 1.0))
        assert(path.getPoint(1.0) == Pose(2.0, 2.0))
        assert(path.getLength() == 2 * sqrt(2.0))
        assert(path.getLookaheadPoint(Pose(0.0, 0.0), 1.0) == Pose(sqrt(2.0)/2.0, sqrt(2.0)/2.0))
    }

    @Test
    fun hermitePath() {
        val path = HermitePath(
            Pose(0.0, 0.0),
            Pose(1.0, 1.0),
            startVelocity = Pose(1.0, 0.0),
            endVelocity = Pose(1.0, 0.0))
        assert(path.getPoint(0.0) == Pose(0.0, 0.0))
        assert(path.getPoint(0.5) == Pose(0.5, 0.5))
        assert(path.getPoint(1.0) == Pose(1.0, 1.0))
//        assert(path.getLength() == sqrt(2.0))
        assert (path.getLookaheadPoint(Pose(0.0, 0.0), 1.0)?.roughlyEquals(Pose(0.66873797,0.743498165)) == true)
    }
}
