package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Pose2d beginPos = new Pose2d(-38, -60, Math.PI * 1.5);
        Pose2d basketPos = new Pose2d(-59.5, -53.25, Math.PI * 1.75);
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(basketPos, Math.PI * 1.2)
                .splineToLinearHeading(new Pose2d(-37, -33.5, Math.PI * 0.75), Math.PI / 2)
                .splineToLinearHeading(basketPos, Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-39, -34, Math.PI * 0.75), Math.PI / 2)
                .splineToLinearHeading(basketPos, Math.PI / 2)
                .splineToLinearHeading(new Pose2d(-48, -35, Math.PI * 0.75), Math.PI / 2)
                .splineToLinearHeading(basketPos, Math.PI / 2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}