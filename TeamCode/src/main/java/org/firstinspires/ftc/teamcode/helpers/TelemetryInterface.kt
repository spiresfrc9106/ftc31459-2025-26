package org.firstinspires.ftc.teamcode.helpers

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.localization.Pose

/** Class used to update both the phone telemetry and the dashboard telemetry. */
class TelemetryInterface(val phone : Telemetry, val dashboard : Telemetry) {
    var enableDashboard = true

    fun addData(name : String, data : Any, useDashboard : Boolean? = null) {
        phone.addData(name, data)
        if (useDashboard == null) {
            if (enableDashboard) addDashboardData(name, data)
        } else if (useDashboard) {
            addDashboardData(name, data)
        }
    }

    fun addLine(line : String, useDashboard : Boolean? = null) {
        phone.addLine(line)
        if (useDashboard == null) {
            if (enableDashboard) dashboard.addLine(line)
        } else if (useDashboard) {
            dashboard.addLine(line)
        }
    }

    fun update() {
        dashboard.update()
        phone.update()
    }

    fun addDashboardData(name: String, data: Any) {
        // Convert Pose objects to separate X Y and Theta for graphing
        if (data is Pose) {
            dashboard.addData("$name X", "%.3f".format(data.x))
            dashboard.addData("$name Y", "%.3f".format(data.y))
            dashboard.addData("$name Heading", "%.3f".format(data.heading))
        } else {
            dashboard.addData(name, data)
        }
    }
}