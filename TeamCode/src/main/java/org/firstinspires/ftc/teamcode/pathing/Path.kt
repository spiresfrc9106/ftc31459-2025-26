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
        CUBIC_HERMITE
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
     * Gets the point at the start of the path
     * @return The point at the start of the path
     */
    fun getStartPose(): Pose {
        return startPose
    }

    /**
     * Gets the point at the end of the path
     * @return The point at the end of the path
     */
    fun getEndPose(): Pose {
        return endPose
    }

    /**
     * Gets the heading interpolation mode for the path
     * @return The heading interpolation mode
     */
    fun getHeadingInterpolationMode(): HeadingInterpolationMode {
        return headingInterpolationMode
    }

    /**
     * Sets the start pose of the path
     * @param pose The start pose to set
     */
    fun setStartPose(pose: Pose) {
        startPose = pose
    }

    /**
     * Sets the end pose of the path
     * @param pose The end pose to set
     */
    fun setEndPose(pose: Pose) {
        endPose = pose
    }

    /**
     * Sets the heading interpolation mode for the path
     * which determines how the target heading is calculated
     * @param mode The heading interpolation mode to set
     */
    fun setHeadingInterpolationMode(mode: HeadingInterpolationMode) {
        headingInterpolationMode = mode
    }

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
     * Gets the closest point on the path to the given point
     * @param point The point to find the closest point to
     * @return The closest point on the path to the given point
     */
    fun getClosestPoint(point: Pose): Pose

    /**
     * Gets the parameter t of the closest point on the path to the given point
     * @param point The point to find the closest point to
     * @return The parameter t of the closest point on the path to the given point
     */
    fun getClosestPointT(point: Pose): Double

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
            HeadingInterpolationMode.CUBIC_HERMITE -> {
                TODO("Not yet implemented")
            }
        }
    }
}
