package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Tom and Sammy: Changed this to the initial pose.
        // Tom and Sammy: You really wanted the y 24 to be -24
        Pose2d initialPose = new Pose2d(new Vector2d(-68,-24), Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 25, Math.toRadians(60), Math.toRadians(9), 11)
                .followTrajectorySequence(
                        drive -> drive.trajectorySequenceBuilder(initialPose)
                                // Tom and Sammy: You really wanted y=-12
                                .splineTo(new Vector2d(-6, 6), Math.toRadians(135))
                                // Tom and Sammy, removing the Vel and Accel constraints: .splineTo(new Vector2d(12, 12), Math.toRadians(135), endVelConstraint, endAccelConstraint )
                                // Tom and Sammy, can't splineTo where we are, use .turn: .splineTo(new Vector2d(12, 12), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                                //.turn(Math.toRadians(-45), new TurnConstraints(15,-15, 15)) // This is a relative angle
                                .build()


                );
                /*
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .build());

                 */


        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}