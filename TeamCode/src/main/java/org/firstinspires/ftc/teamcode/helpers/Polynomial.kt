package org.firstinspires.ftc.teamcode.helpers

// Note: coeffs are least to most significant
class Polynomial(coeffs : Array<Double>) {
    val coefficients = coeffs

    // --- Basic operations ---

    // Evaluates as a nested polynomial
    fun eval(x : Double, coeffs : Array<Double> = coefficients) : Double {
        var value: Double = 0.0
        for (coeff in coeffs.reversed()) {
            value = coeff + (x*value)
        }
        return value
    }

    // --- Calculus ---

    fun derivative() : Array<Double> {
        val der = Array<Double>(coefficients.size - 1) {0.0}
        for (i in 1..<coefficients.size) {
            der[i - 1] = coefficients[i] * i
        }
        return der
    }

    fun antiDerivative() : Array<Double> {
        val antiDer = Array<Double>(coefficients.size + 1) {0.0}
        for (i in coefficients.indices) {
            antiDer[i + 1] = coefficients[i] / (i+1)
        }
        return antiDer
    }

    fun derEval(x : Double) : Double {
        val der = derivative()
        return eval(x, der)
    }

    fun finiteInt(x1 : Double, x2: Double) : Double {
        val antiDer = antiDerivative()
        return (eval(x2, antiDer) - eval(x1, antiDer));
    }

}