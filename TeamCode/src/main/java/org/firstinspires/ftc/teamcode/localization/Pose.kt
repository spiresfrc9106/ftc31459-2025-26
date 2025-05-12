package org.firstinspires.ftc.teamcode.localization

import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Represents a 2D pose with x, y coordinates and a heading (angle).
 * Provides methods for vector operations, distance calculations, and equality checks.
 */
class Pose(var x: Double, var y: Double, var heading: Double) {
    // Constructor overloads
    constructor(x: Double, y: Double) : this(x, y, 0.0)
    constructor() : this(0.0, 0.0, 0.0)

    // Getters
    fun getX(): Double { return x }
    fun getY(): Double { return y }
    fun getHeading(): Double { return heading }

    // Setters
    fun setX(x: Double) { this.x = x }
    fun setY(y: Double) { this.y = y }
    fun setHeading(heading: Double) { this.heading = heading }

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

    fun roughlyEquals(other: Pose, tolerance: Double): Boolean {
        return this.distanceTo(other) < tolerance
    }

    fun roughlyEquals(other: Pose): Boolean {
        return this.roughlyEquals(other, 0.001)
    }

    override fun toString(): String {
        return "($x, $y, $heading)"
    }

    fun copy(): Pose {
        return Pose(x, y, heading)
    }
}
