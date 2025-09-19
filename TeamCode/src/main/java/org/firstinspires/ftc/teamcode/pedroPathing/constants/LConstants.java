package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .001989436789;
        ThreeWheelIMUConstants.strafeTicksToInches = .001989436789;
        ThreeWheelIMUConstants.turnTicksToInches = .001989436789;

        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "rightRear";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftRear";
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.leftY = 6;
        ThreeWheelIMUConstants.rightY = -6;
        ThreeWheelIMUConstants.strafeX = -7.6;
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}




