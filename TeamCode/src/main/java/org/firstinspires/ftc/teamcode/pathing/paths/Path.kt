package org.firstinspires.ftc.teamcode.pathing.paths

import org.firstinspires.ftc.teamcode.localization.Pose


/**
 * Path interface representing a path in 2D space
 */
interface Path {
    /**
     * Enum for heading interpolation modes
     */
    enum class HeadingInterpolationMode {
        LINEAR,
    }

    /**
     * Enum for heading interpolation mode
     */
    var headingInterpolationMode: HeadingInterpolationMode

    /**
     * The start and end poses of the path
     */
    var startPose: Pose

    /**
     * The end pose of the path
     */
    var endPose: Pose

    /**
     * Gets the length of the path
     * @return The length of the path
     */
    fun getLength(): Double

    /**
     * Gets the heading at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The heading at the given parameter
     */
    fun getHeading(t: Double): Double

    /**
     * Gets the point at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The point at the given parameter
     */
    fun getPoint(t: Double): Pose

    /**
     * Gets the derivative (tangent) at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The tangent vector at the given parameter
     */
    fun getTangent(t: Double): Pose

    /**
     * Gets the normal vector at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The normal vector at the given parameter
     */
    fun getNormal(t: Double): Pose

    /**
     * Gets the curvature at the given parameter t
     * @param t The parameter value in the range [0, 1]
     * @return The curvature at the given parameter
     */
    fun getCurvature(t: Double): Double

    /**
     * Gets the time of intersection between the path and a circle with radius lookaheadDistance and center at position.
     * If there are multiple intersections, the largest t value is returned, if there are no intersections, -1 is returned.
     * @param position The center of the circle
     * @param lookaheadDistance The radius of the circle
     * @return The time parameter t at which the circle intersects the path
     */
    fun getLookaheadPointT(position: Pose, lookaheadDistance: Double): Double

    /**
     * Gets the point of intersection between the path and a circle with radius lookaheadDistance and center at position
     * If there are multiple intersections, the largest t value is used to get the point. If there are no intersections, the start point is returned.
     * @param position The center of the circle
     * @param lookaheadDistance The radius of the circle
     * @return The point at which the circle intersects the path
     */
    fun getLookaheadPoint(position: Pose, lookaheadDistance: Double): Pose {
        val t = getLookaheadPointT(position, lookaheadDistance)
        if (t == -1.0) {
            // If there is no intersection, return the start point
            return getPoint(0.0)
        }
        return getPoint(t)
    }

    /**
     * Gets the heading goal at the given parameter t based on the interpolation mode
     * @param t The parameter value in the range [0, 1]
     * @return The heading goal at the given parameter
     */
    fun getHeadingGoal(t: Double): Double {
        when (headingInterpolationMode) {
            HeadingInterpolationMode.LINEAR -> {
                return startPose.heading + (endPose.heading - startPose.heading) * t
            }
        }
    }
}
