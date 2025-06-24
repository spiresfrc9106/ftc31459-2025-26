package org.firstinspires.ftc.teamcode.pathing.paths

import org.firstinspires.ftc.teamcode.localization.Pose

/**
 * Represents a compound path made up of multiple paths.
 * The paths are assumed to be connected end-to-end, meaning the end pose of one path
 * should match the start pose of the next path in the list.
 * @param paths A list of paths that make up the compound path.
 */
class CompoundPath(val paths: List<Path>) : Path {
    constructor(vararg paths: Path) : this(paths.toList())

    override var startPose: Pose = paths.first().startPose
    override var endPose: Pose = paths.last().endPose
    override var headingInterpolationMode: Path.HeadingInterpolationMode = Path.HeadingInterpolationMode.LINEAR

    init {
        // Validate that the paths are connected end-to-end
        for (i in 0 until paths.size - 1) {
            if (paths[i].endPose != paths[i + 1].startPose) {
                throw IllegalArgumentException("Paths are not connected end-to-end")
            }
        }
    }

    override fun getLength(): Double {
        return paths.sumOf { it.getLength() }
    }

    override fun getHeading(t: Double): Double {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getHeading(localT)
    }

    override fun getPoint(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getPoint(localT)
    }

    override fun getTangent(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getTangent(localT)
    }

    override fun getNormal(t: Double): Pose {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getNormal(localT)
    }

    override fun getCurvature(t: Double): Double {
        val pathIndex = (t * paths.size).toInt().coerceIn(0, paths.size - 1)
        val localT = t * paths.size - pathIndex
        return paths[pathIndex].getCurvature(localT)
    }

    override fun getLookaheadPointT(position: Pose, lookaheadDistance: Double): Double? {
        // Return the farthest intersection point along the compound path
        for (i in paths.size - 1 downTo 0) {
            val lookaheadPointT = paths[i].getLookaheadPointT(position, lookaheadDistance)
            if (lookaheadPointT != null) {
                // Calculate the global t value for the intersection point
                val globalT = (i + lookaheadPointT) / paths.size
                return globalT
            }
        }
        return null // No intersection found
    }

    override fun getClosestPointT(position: Pose): Double {
        // Find the closest point on each path and return the closest one
        var closestT = -1.0
        var minDistance = Double.MAX_VALUE

        for (path in paths) {
            val t = path.getClosestPointT(position)
            val point = path.getPoint(t)
            val distance = position.distanceTo(point)

            if (distance < minDistance) {
                minDistance = distance
                closestT = (t + paths.indexOf(path)) / paths.size
            }
        }

        return closestT
    }

    class Builder {
        private val paths = mutableListOf<Path>()

        fun addPath(path: Path): Builder {
            paths.add(path)
            return this
        }

        fun build(): CompoundPath {
            return CompoundPath(paths)
        }
    }

    class PolyLineBuilder {
        private val points = mutableListOf<Pose>()

        fun addPoint(point: Pose): PolyLineBuilder {
            points.add(point)
            return this
        }

        fun build(): CompoundPath {
            if (points.size < 2) {
                throw IllegalArgumentException("At least two points are required to create a polyline")
            }
            val paths = mutableListOf<Path>()
            for (i in 0 until points.size - 1) {
                paths.add(LinearPath(points[i], points[i + 1]))
            }
            return CompoundPath(paths)
        }
    }
}
