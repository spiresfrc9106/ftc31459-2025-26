package org.firstinspires.ftc.teamcode.helpers

// Note: coeffs are least to most significant
class Polynomial(coeffs: Array<Double>) {
    val coefficients = coeffs

    // --- Basic operations ---

    // Evaluates as a nested polynomial
    fun eval(x: Double, coeffs: Array<Double> = coefficients) : Double {
        var value: Double = 0.0
        for (coeff in coeffs.reversed()) {
            value = coeff + (x*value)
        }
        return value
    }

    // --- Transformations ---

    fun scale(a: Double) : Polynomial {
        val newCoeffs = Array(coefficients.size) {0.0}
        for (i in coefficients.indices) {
            newCoeffs[i] = coefficients[i] * a
        }
        return Polynomial(newCoeffs)
    }

    fun square() : Polynomial {
        val newCoeffs = Array(coefficients.size * 2 - 1) {0.0}
        for (i in coefficients.indices) {
            for (j in coefficients.indices) {
                val newIndex = i+j // Index is essentially equivalent to degree
                val newCoeff = coefficients[i] * coefficients[j]
                newCoeffs[newIndex] += newCoeff
            }
        }
        return Polynomial(newCoeffs)
    }

    // --- Calculus ---

    fun derivative() : Polynomial {
        val der = Array(coefficients.size - 1) {0.0}
        for (i in 1..<coefficients.size) {
            der[i - 1] = coefficients[i] * i
        }
        return Polynomial(der)
    }

    fun antiDerivative() : Polynomial {
        val antiDer = Array<Double>(coefficients.size + 1) {0.0}
        for (i in coefficients.indices) {
            antiDer[i + 1] = coefficients[i] / (i+1)
        }
        return Polynomial(antiDer)
    }

    fun derEval(x: Double) : Double {
        val der = derivative()
        return eval(x, der.coefficients)
    }

    fun finiteInt(x1: Double, x2: Double) : Double {
        val antiDer = antiDerivative()
        return (eval(x2, antiDer.coefficients) - eval(x1, antiDer.coefficients))
    }

}