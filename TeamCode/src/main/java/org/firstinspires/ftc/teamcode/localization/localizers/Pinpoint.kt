package org.firstinspires.ftc.teamcode.localization.localizers

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.HardwareNames
import org.firstinspires.ftc.teamcode.localization.Localizer
import org.firstinspires.ftc.teamcode.localization.Pose
import org.firstinspires.ftc.teamcode.localization.constants.PinpointConstants

class Pinpoint (hardwareMap: HardwareMap, startPose: Pose = Pose()) : Localizer {
    // Current pose, velocity, and acceleration
    override var pose: Pose = startPose
    override var velocity: Pose = Pose()
    /** Acceleration is not directly available from Pinpoint. */
    override var acceleration: Pose = Pose()

    // Pinpoint driver
    private val pinpoint = hardwareMap.get(GoBildaPinpointDriver::class.java, HardwareNames.PINPOINT)

    init {
        pinpoint.setOffsets(PinpointConstants.Y_POD_OFFSET, PinpointConstants.X_POD_OFFSET, DistanceUnit.MM)
        pinpoint.setEncoderResolution(PinpointConstants.ENCODER_RESOLUTION)
        pinpoint.setEncoderDirections(PinpointConstants.Y_ENCODER_DIRECTION, PinpointConstants.X_ENCODER_DIRECTION)
        pinpoint.recalibrateIMU()
        pinpoint.setPosition(Pose2D(DistanceUnit.CM, startPose.x, startPose.y, AngleUnit.RADIANS, startPose.heading))
    }

    override fun update(deltaTime: Double) {
        pinpoint.update()
        // Update pose and velocity based on the pinpoint data
        // X and Y are swapped because the Pinpoint's coordinate system is flipped
        pose = Pose(
            pinpoint.getPosY(DistanceUnit.CM),
            pinpoint.getPosX(DistanceUnit.CM),
            pinpoint.getHeading(AngleUnit.RADIANS)
        )
        velocity = Pose(
            pinpoint.getVelY(DistanceUnit.CM),
            pinpoint.getVelX(DistanceUnit.CM),
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        )
    }

    override fun reset(pose: Pose) {
        this.pose = pose
        pinpoint.setPosition(Pose2D(DistanceUnit.CM, pose.x, pose.y, AngleUnit.RADIANS, pose.heading))
    }
}
