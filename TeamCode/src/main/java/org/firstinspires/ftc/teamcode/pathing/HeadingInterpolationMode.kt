package org.firstinspires.ftc.teamcode.pathing

enum class HeadingInterpolationMode {
    /**
     * Interpolates the heading linearly between the start and end points of the path.
     */
    LINEAR,
    /**
     * Interpolates the heading using a cubic Hermite spline, which provides a smoother transition
     * between the start and end headings.
     */
    CUBIC_HERMITE,
}