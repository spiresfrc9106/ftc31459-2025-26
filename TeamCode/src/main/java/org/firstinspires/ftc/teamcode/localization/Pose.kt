package org.firstinspires.ftc.teamcode.localization

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Represents a 2D pose with x, y coordinates and a heading (angle).
 * Provides methods for vector operations, distance calculations, and equality checks.
 */
class Pose(var x: Double = 0.0, var y: Double = 0.0, var heading: Double = 0.0) {
    // Vector operations
    fun add(other: Pose) {
        this.x += other.x
        this.y += other.y
        this.heading += other.heading
    }

    fun subtract(other: Pose) {
        this.x -= other.x
        this.y -= other.y
        this.heading -= other.heading
    }

    fun multiply(scalar: Double) {
        this.x *= scalar
        this.y *= scalar
        this.heading *= scalar
    }

    fun divide(scalar: Double) {
        if (scalar != 0.0) {
            this.x /= scalar
            this.y /= scalar
            this.heading /= scalar
        } else {
            throw IllegalArgumentException("Cannot divide by zero")
        }
    }

    // Other utility methods
    fun distanceTo(other: Pose): Double {
        return sqrt((other.x - this.x).pow(2.0) + (other.y - this.y).pow(2.0))
    }

    fun roughlyEquals(other: Pose, tolerance: Double = 0.001): Boolean {
        return this.distanceTo(other) < tolerance
    }

    override fun toString(): String {
        return "(x=${"%.3f".format(x)}, y=${"%.3f".format(y)}, heading=${"%.3f".format(heading)})"
    }

    fun copy(): Pose {
        return Pose(x, y, heading)
    }

    fun getPose2D() : Pose2D {
        return Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, heading)
    }
}
