package com.example.mangojuice;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MangoJuice {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-36, -57, Math.toRadians(0)))
                        .lineToConstantHeading(new Vector2d(-35, -50))
                       .splineToLinearHeading(new Pose2d(-50,-50,Math.toRadians(45)),Math.toRadians(-40))
                        .lineToConstantHeading(new Vector2d(-54, -54)) //preload dropped

                        .splineToLinearHeading(new Pose2d(-48,-36,Math.toRadians(90)),Math.toRadians(90)) //to B1
                        .forward(3) //pick up
                        .back(4) //go back
                        .lineToLinearHeading(new Pose2d(-50,-50,Math.toRadians(45))) //away from Bs to bucket
                        .lineToConstantHeading(new Vector2d(-54, -54)) //forward to bucket
                        .forward(3) //a bit away from bucket idk, prep for next cycle

                        .splineToLinearHeading(new Pose2d(-58,-36,Math.toRadians(90)),Math.toRadians(90)) //to B2
                        .forward(3) //pick up
                        .back(7) //go back
                        .lineToLinearHeading(new Pose2d(-50,-50,Math.toRadians(45))) //away from Bs to bucket
                        .lineToConstantHeading(new Vector2d(-54, -54)) //forward to bucket
                        .forward(3) //a bit away from bucket idk, prep for next cycle






                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}