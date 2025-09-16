package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep) // BotBuilder... say that again...
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(61, 0, Math.toRadians(180)))
                        .forward(0.01)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        .build());

        Image background = null;
        try {
            background = ImageIO.read(new File("MeepMeepTesting/src/main/java/com/example/meepmeeptesting/decode.png"));
        } catch (IOException ignored) {}
        assert background != null;

        meepMeep.setBackground(background)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}