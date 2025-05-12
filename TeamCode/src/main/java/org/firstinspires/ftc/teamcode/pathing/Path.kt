package org.firstinspires.ftc.teamcode.pathing

import org.firstinspires.ftc.teamcode.localization.Pose

interface Path {
    /**
     * Gets the point at the start of the path
     * @return The point at the start of the path
     */
    fun getStartPose(): Pose

    /**
     * Gets the point at the end of the path
     * @return The point at the end of the path
     */
    fun getEndPose(): Pose

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
     * Gets the heading goal at the given parameter t based on the interpolation mode
     * @param t The parameter value in the range [0, 1]
     * @return The heading goal at the given parameter
     */
    fun getHeadingGoal(t: Double): Double

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
     * Gets the heading interpolation mode for the path
     * @return The heading interpolation mode
     */
    fun getHeadingInterpolationMode(): HeadingInterpolationMode

    /**
     * Sets the heading interpolation mode for the path
     * which determines how the target heading is calculated
     * @param mode The heading interpolation mode to set
     */
    fun setHeadingInterpolationMode(mode: HeadingInterpolationMode)
}