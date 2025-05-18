package org.firstinspires.ftc.teamcode.pathing

import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.pathing.Path.HeadingInterpolationMode


/**
 * LinearPath class representing a straight line path in 2D space
 * @param startPose The starting pose of the path
 * @param endPose The ending pose of the path
 */
class LinearPath (override var startPose: Pose, override var endPose: Pose) : Path {
    // Constructor overloads
    constructor(startX: Double, startY: Double, endX: Double, endY: Double) : this(Pose(startX, startY), Pose(endX, endY))
    constructor() : this(Pose(), Pose())

    // Enum for heading interpolation mode
    override var headingInterpolationMode: HeadingInterpolationMode = HeadingInterpolationMode.LINEAR

    override fun getLength(): Double {
        return startPose.distanceTo(endPose)
    }

    override fun getPoint(t: Double): Pose {
        val x = startPose.x + (endPose.x - startPose.x) * t
        val y = startPose.y + (endPose.y - startPose.y) * t
        return Pose(x, y)
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

    override fun getClosestPointT(point: Pose): Double {
        val dx = endPose.x - startPose.x
        val dy = endPose.y - startPose.y

        val dotProduct = (point.x - startPose.x) * dx + (point.y - startPose.y) * dy
        val lengthSquared = dx * dx + dy * dy

        return if (lengthSquared != 0.0) {
            (dotProduct / lengthSquared).coerceIn(0.0, 1.0)
        } else {
            0.0
        }
    }

    override fun getClosestPoint(point: Pose): Pose {
        val t = getClosestPointT(point)
        return getPoint(t)
    }
}
