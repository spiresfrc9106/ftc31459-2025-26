package org.firstinspires.ftc.teamcode.pathing.paths

import org.firstinspires.ftc.teamcode.helpers.Polynomial
import org.firstinspires.ftc.teamcode.helpers.PolynomialUtils
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.Path.HeadingInterpolationMode
import kotlin.math.pow

/**
 * HermitePath class representing a Hermite curve
 * @param startPose The starting pose of the path
 * @param endPose The ending pose of the path
 * @param startVelocity The starting velocity vector [x,y]
 * @param endVelocity The ending velocity vector [x,y]
 */
class HermitePath(override var startPose: Pose, override var endPose: Pose,
                  startVelocity: Pose = Pose(),
                  endVelocity: Pose = Pose()) : Path {

    // Enum for heading interpolation mode
    override var headingInterpolationMode: HeadingInterpolationMode = HeadingInterpolationMode.LINEAR

    // Hermite basis functions
    private val basis00 = Polynomial(arrayOf(1.0, 0.0, -3.0, 2.0))
    private val basis01 = Polynomial(arrayOf(0.0, 1.0, -2.0, 1.0))
    private val basis10 = Polynomial(arrayOf(0.0, 0.0, 3.0, -2.0))
    private val basis11 = Polynomial(arrayOf(0.0, 0.0, -1.0, 1.0))

    private val xHermite = PolynomialUtils.addPolynomials(arrayOf(basis00.vScale(startPose.x), basis01.vScale(startVelocity.x),
                                                          basis10.vScale(endPose.x), basis11.vScale(endVelocity.x)))
    private val yHermite = PolynomialUtils.addPolynomials(arrayOf(basis00.vScale(startPose.y), basis01.vScale(startVelocity.y),
                                                          basis10.vScale(endPose.y), basis11.vScale(endVelocity.y)))

    // Compound path for simplified calculations
    private val resolution: Int = 100 // Resolution for the compound path
    val compoundPath: Path = createCompoundPath(resolution)

    override fun getLength(): Double {
        // Use the compound path to estimate the length
        return compoundPath.getLength()
    }

    override fun getLengthSoFar(t: Double): Double {
        // Use the compound path to estimate the length so far
        return compoundPath.getLengthSoFar(t)
    }

    override fun getHeading(t: Double): Double {
        when (headingInterpolationMode) {
            HeadingInterpolationMode.LINEAR -> {
                return startPose.heading + (endPose.heading - startPose.heading) * t
            }
        }
    }

    override fun getPoint(t: Double): Pose {
        val x = xHermite.eval(t)
        val y = yHermite.eval(t)
        val heading = getHeading(t)
        return Pose(x, y, heading)
    }

    override fun getTangent(t: Double): Pose {
        val xDer = xHermite.derivative()
        val yDer = yHermite.derivative()
        val dxdt = xDer.derEval(t)
        val dydt = yDer.derEval(t)
        return Pose(dxdt, dydt)
    }

    override fun getNormal(t: Double): Pose {
        val tangent = getTangent(t)
        return Pose(-tangent.y, tangent.x)
    }

    override fun getCurvature(t: Double): Double {
        val xDer = xHermite.derEval(t)
        val yDer = yHermite.derEval(t)
        val xDer2 = xHermite.nDerEval(t, 2) // Second derivative
        val yDer2 = yHermite.nDerEval(t, 2)

        val numerator = xDer * xDer2 - yDer2 * yDer
        var denominator = xDer.pow(2.0) + yDer.pow(2.0)
        denominator = denominator.pow(1.5)

        return (numerator / denominator)
    }

    override fun getLookaheadPointT(position: Pose, lookaheadDistance: Double): Double? {
        return compoundPath.getLookaheadPointT(position, lookaheadDistance)
    }

    override fun getClosestPointT(position: Pose): Double {
        return compoundPath.getClosestPointT(position)
    }

    fun createCompoundPath(resolution: Int): Path {
        // Create a compound linear path to represent the Hermite path
        val builder = LinearPath.Builder()
        for (i in 0..resolution) {
            val t = i.toDouble() / resolution
            val point = getPoint(t)
            builder.addPoint(point)
        }
        return builder.build()
    }


    class Builder {
        private val points = mutableListOf<Pose>()
        private val velocities = mutableListOf<Pose?>()

        fun addPoint(point: Pose, velocity: Pose? = null): Builder {
            points.add(point)
            velocities.add(velocity)
            return this
        }

        // Throw in points, not velocities
        private fun getAvVelocity(p0: Pose, p1: Pose, p2: Pose): Pose {
            val dx1 = p1.x - p0.x
            val dy1 = p1.y - p0.y
            val dx2 = p2.x - p1.x
            val dy2 = p2.y - p1.y

            return Pose((dx1 + dx2)/2, (dy1 + dy2)/2)
        }

        fun build(): Path {
            if (points.size < 2) {
                throw IllegalArgumentException("At least two points are required to create a HermitePath")
            }
            else if (points.size == 2) {
                return HermitePath(points[0], points[1], velocities[0]?: Pose(), velocities[1]?: Pose())
            }
            else {
                val paths = mutableListOf<HermitePath>()
                for (i in 0..points.size - 2) {
                    val prevPoint = if (i > 0) points[i-1] else null
                    val startPoint = points[i]
                    val endPoint = points[i+1]
                    val nextPoint = if (i < points.size - 2) points[i+2] else null

                    val defaultStartVel = if (i > 0) getAvVelocity(prevPoint!!, startPoint, endPoint) else Pose()
                    val defaultEndVel = if (i < points.size - 2) getAvVelocity(startPoint, endPoint, nextPoint!!) else Pose()

                    paths.add(HermitePath(startPoint, endPoint, velocities[i] ?: defaultStartVel, velocities[i+1] ?: defaultEndVel))
                }
                return CompoundPath(paths)
            }
        }
    }
}
