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
                  startVelocity: Array<Double> = arrayOf(-1.0, -1.0),
                  endVelocity: Array<Double> = arrayOf(-1.0, -1.0)) : Path {

    // Enum for heading interpolation mode
    override var headingInterpolationMode: HeadingInterpolationMode = HeadingInterpolationMode.LINEAR

    // Hermite basis functions
    val basis00 = Polynomial(arrayOf(1.0, 0.0, -3.0, 2.0))
    val basis01 = Polynomial(arrayOf(0.0, 1.0, -2.0, 1.0))
    val basis10 = Polynomial(arrayOf(0.0, 0.0, -3.0, 2.0))
    val basis11 = Polynomial(arrayOf(0.0, 0.0, -1.0, 1.0))

    val xHermite = PolynomialUtils.addPolynomials(arrayOf(basis00.scale(startPose.x), basis01.scale(startVelocity[0]),
                                                          basis10.scale(endPose.x), basis11.scale(endVelocity[0])))
    val yHermite = PolynomialUtils.addPolynomials(arrayOf(basis00.scale(startPose.y), basis01.scale(startVelocity[1]),
                                                          basis10.scale(endPose.y), basis11.scale(endVelocity[1])))

    override fun getLength(): Double {
        var xDer = xHermite.derivative()
        var yDer = yHermite.derivative()
        xDer = xDer.square()
        yDer = yDer.square()

        TODO("figure out how to do square root ajshdpfapjds")

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

    override fun getLookaheadPointT(position: Pose, lookaheadDistance: Double): Double {
        TODO("Not yet implemented")
    }

    override fun getClosestPointT(position: Pose): Double {
        TODO("Not yet implemented")
    }

}