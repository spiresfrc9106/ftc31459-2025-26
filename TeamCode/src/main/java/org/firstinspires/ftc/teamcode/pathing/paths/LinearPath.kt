package org.firstinspires.ftc.teamcode.pathing.paths

import org.firstinspires.ftc.teamcode.helpers.Angle
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.paths.Path.HeadingInterpolationMode
import kotlin.math.sqrt


/**
 * LinearPath class representing a straight line path in 2D space
 * @param startPose The starting pose of the path
 * @param endPose The ending pose of the path
 */
class LinearPath (override var startPose: Pose = Pose(), override var endPose: Pose = Pose()) :
    Path {
    // Constructor overloads
    constructor(startX: Double, startY: Double, endX: Double, endY: Double) : this(Pose(startX, startY), Pose(endX, endY))

    // Enum for heading interpolation mode
    override var headingInterpolationMode: HeadingInterpolationMode = HeadingInterpolationMode.LINEAR

    override fun getLength(): Double {
        return startPose.distanceTo(endPose)
    }

    override fun getHeading(t: Double): Double {
        when (headingInterpolationMode) {
            HeadingInterpolationMode.LINEAR -> {
                val delta = Angle.normalizeRadians(endPose.heading - startPose.heading)
                return startPose.heading + delta * t
            }
        }
    }

    override fun getPoint(t: Double): Pose {
        val x = startPose.x + (endPose.x - startPose.x) * t
        val y = startPose.y + (endPose.y - startPose.y) * t
        val heading = getHeading(t)
        return Pose(x, y, heading)
    }

    override fun getTangent(t: Double): Pose {
        val dx = endPose.x - startPose.x
        val dy = endPose.y - startPose.y
        val length = getLength()
        return if (length != 0.0) {
            Pose(dx / length, dy / length)
        } else {
            Pose(0.0, 0.0)
        }
    }

    override fun getNormal(t: Double): Pose {
        val tangent = getTangent(t)
        return Pose(-tangent.y, tangent.x)
    }

    override fun getCurvature(t: Double): Double {
        return 0.0 // Linear paths have zero curvature
    }

    override fun getLookaheadPointT(position: Pose, lookaheadDistance: Double): Double? {
        val dx = endPose.x - startPose.x
        val dy = endPose.y - startPose.y
        val fx = startPose.x - position.x
        val fy = startPose.y - position.y

        // Calculate coefficients for the quadratic equation
        val a = dx * dx + dy * dy
        val b = 2 * (fx * dx + fy * dy)
        val c = fx * fx + fy * fy - lookaheadDistance * lookaheadDistance

        // If the discriminant is negative, there are no real solutions
        val discriminant = b * b - 4 * a * c
        if (discriminant < 0) {
            return null // No intersection
        }

        // Calculate the two possible t values using the quadratic formula
        var t1 = (-b + sqrt(discriminant)) / (2 * a)
        var t2 = (-b - sqrt(discriminant)) / (2 * a)

        // Coerce t values to the range [0, 1]
        t1 = t1.coerceIn(0.0, 1.0)
        t2 = t2.coerceIn(0.0, 1.0)

        return if (t1 > t2) t1 else t2 // Return the larger t value
    }

    override fun getClosestPointT(position: Pose): Double {
        val dx = endPose.x - startPose.x
        val dy = endPose.y - startPose.y
        val fx = position.x - startPose.x
        val fy = position.y - startPose.y

        // Calculate the projection of (fx, fy) onto (dx, dy)
        val dotProduct = fx * dx + fy * dy
        val lengthSquared = dx * dx + dy * dy

        if (lengthSquared == 0.0) {
            return 0.0 // The path is a point
        }

        var t = dotProduct / lengthSquared
        t = t.coerceIn(0.0, 1.0) // Coerce to [0, 1]

        return t
    }
}
