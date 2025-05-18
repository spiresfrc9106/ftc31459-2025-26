package org.firstinspires.ftc.teamcode.pathing

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
     * Gets the parameter t of the closest point on the path to the given point
     * @param point The point to find the closest point to
     * @return The parameter t of the closest point on the path to the given point
     */
    fun getClosestPointT(point: Pose): Double

    /**
     * Gets the closest point on the path to the given point
     * @param point The point to find the closest point to
     * @return The closest point on the path to the given point
     */
    fun getClosestPoint(point: Pose): Pose

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
