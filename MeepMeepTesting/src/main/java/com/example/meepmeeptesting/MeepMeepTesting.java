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

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 25, Math.toRadians(60), Math.toRadians(9), 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-20, 0, 0))
                .splineTo(new Vector2d(-4, -5), Math.toRadians(60))
                .splineTo(new Vector2d(0, 15), Math.toRadians(90) )
                .splineTo(new Vector2d(0, 20), Math.toRadians(90) )
                .build());



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