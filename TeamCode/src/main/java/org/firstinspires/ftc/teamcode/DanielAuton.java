package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

import java.util.ArrayList;

@Config
@Autonomous(name = "--DanielAuton", group = "Autonomous")
public class DanielAuton extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

//    public static double SAMPLE_DISTANCE_OFFSET = -10;
//    public static double SAMPLE_ANGLE_COEFF = 0.8;

    public static double BASKET_X = 4.5;
    public static double BASKET_Y = 20.5;
    public static double BASKET_R = Math.PI * 1.75;

    public static double SAMPLE_1_X = 16;
    public static double SAMPLE_1_Y = 10.5;
    public static double SAMPLE_1_R = 0;

    public static double SAMPLE_2_X = 16;
    public static double SAMPLE_2_Y = 21;
    public static double SAMPLE_2_R = 0;

    public static double SAMPLE_3_X = 21;
    public static double SAMPLE_3_Y = 21;
    public static double SAMPLE_3_R = 0.47;

    public static double SUBMERSIBLE_X = 16;
    public static double SUBMERSIBLE_Y = 20;
    public static double SUBMERSIBLE_R = 0;

    public static double END_X = 16;
    public static double END_Y = 10;
    public static double END_R = 0;

    private final Pose2d beginPos = new Pose2d(0,  0, 0);
    private final Pose2d basketPos = new Pose2d(BASKET_X, BASKET_Y, BASKET_R);

    private final Pose2d sample1Pos = new Pose2d(SAMPLE_1_X, SAMPLE_1_Y, SAMPLE_1_R);
    private final Pose2d sample2Pos = new Pose2d(SAMPLE_2_X, SAMPLE_2_Y, SAMPLE_2_R);
    private final Pose2d sample3Pos = new Pose2d(SAMPLE_3_X, SAMPLE_3_Y, SAMPLE_3_R);
    private final Pose2d submersiblePos = new Pose2d(SUBMERSIBLE_X, SUBMERSIBLE_Y, SUBMERSIBLE_R);

    private final Pose2d endPos = new Pose2d(END_X, END_Y, END_R);

    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;

    private SampleFinder sampleFinder;

    private double sampleAngle;
    private double sampleDistance;

    private Robot.Alliance alliance = Robot.Alliance.RED;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, beginPos);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        sampleFinder = new SampleFinder(hardwareMap, dashboardTelemetry, new double[]{0, 0, 0});

        intake.getSlideMotor().resetEncoder();
        outtake.getSlideMotor().resetEncoder();

        Action endAction = new ParallelAction(
                goTo(endPos),
                retractOuttake(),
                new Intake.MoveSlideActionRR(intake, Intake.SLIDE_IN_POSITION)
        );

        Action mainAction = new SequentialAction(
                goOuttake(),
                cycleSpikeMark(sample1Pos),
                cycleSpikeMark(sample2Pos),
                cycleSpikeMark(sample3Pos),
                endAction
        );

        Action tickAction = telemetryPacket -> {
            intake.stepSlide(dashboardTelemetry);
            outtake.stepSlide(dashboardTelemetry);

            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
            }
            if (gamepad2.dpad_down) {
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad2.dpad_right) {
                alliance = Robot.Alliance.RED;
            }
            if (gamepad2.dpad_left) {
                alliance = Robot.Alliance.BLUE;
            }

            dashboardTelemetry.update();

            return true;
        };

        intake.setWrist(Intake.WRIST_UP_POSITION);

        while (!opModeIsActive()) {
            dashboardTelemetry.update();
            sampleFinder.get(dashboardTelemetry);
        }

        Actions.runBlocking(new ParallelAction(
                tickAction,
                mainAction
        ));

        while (opModeIsActive()) {
            dashboardTelemetry.update();
        }
    }

    private Action cycleSpikeMark(Pose2d pos) {
        return new SequentialAction(
                new ParallelAction(
                        goTo(pos),
                        retractOuttake(),
                        new Intake.WristActionRR(intake, Intake.WRIST_DOWN_POSITION)
                ),
                new Intake.CollectSampleActionRR(intake),
                transfer(),
                goOuttake()
        );
    }

    private Action retractOuttake () {
        return new SequentialAction(
                new Outtake.SpinToInActionRR(outtake),
                new Outtake.MoveSlideActionRR(outtake, Outtake.DOWN_POSITION)
        );
    }

    private Action transfer() {
        return new SequentialAction(
                new ParallelAction(
                        new Intake.WristActionRR(intake, Intake.WRIST_UP_POSITION),
                        new Intake.MoveSlideActionRR(intake, Intake.SLIDE_IN_POSITION)
                ),
                new Intake.TransferSpinActionRR(intake),
                new Intake.ClearWristActionRR(intake)
        );
    }

    private Action goTo(Pose2d pos) {
        return drive.actionBuilder(beginPos)
                .strafeToLinearHeading(pos.position, pos.heading)
                .build();
    }

    private Action rotateTo(double angle) {
        while (angle < 0) {
            angle += Math.PI * 2;
        }

        while (angle > Math.PI * 2) {
            angle -= Math.PI * 2;
        }

        return drive.actionBuilder(beginPos)
                .turnTo(angle)
                .build();
    }

//    private class rotateToSample implements Action {
//        private Action action = null;
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (action == null) action = rotateTo(SUBMERSIBLE_R + sampleAngle * SAMPLE_ANGLE_COEFF);
//            return action.run(telemetryPacket);
//        }
//    }

    private Action goOuttake () {
        return new SequentialAction(
                new ParallelAction(
                        goTo(basketPos),
                        new Outtake.MoveSlideActionRR(outtake, Outtake.HIGH_BASKET_POSITION),
                        new Outtake.SpinToMidActionRR(outtake)
                ),
                new Outtake.DumpActionRR(outtake)
        );
    }

//    private Action cycleSubmersible () {
//        return new SequentialAction (
//                new ParallelAction(
//                        new SequentialAction(
//                                //goTo(submersiblePos),
//                                new FindSample()
//                        ),
//                        retractOuttake(),
//                        new Intake.WristActionRR(intake, Intake.WRIST_UP_POSITION)
//                ),
//                new ParallelAction(
//                        new rotateToSample(),
//                        CollectSubmersibleSample()
//                ),
//                transfer(),
//                goOuttake()
//        );
//    }

//    private class FindSample implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            ArrayList<Sample> samples = sampleFinder.get(dashboardTelemetry);
//
//            Sample sample = selectSample(samples);
//
//            sampleAngle = sample.getAngle();
//            sampleDistance = sample.getZ();
//
//            return samples.isEmpty();
//        }
//    }

//    private Sample selectSample(ArrayList<Sample> samples) {
//        // Score each sample and select the best one
////        Sample bestSample = null;
////        double bestScore = Double.NEGATIVE_INFINITY;
////        for (Sample sample : samples) {
////            double score = evalSample(sample, samples);
////            if (score > bestScore) {
////                bestSample = sample;
////                bestScore = score;
////            }
////        }
////
////        return bestSample;
//
//        return samples.get(0);
//    }
//
//    private double evalSample(Sample sample, ArrayList<Sample> samples) {
//        Vector2d center = new Vector2d(20, 30);
//
//        if (alliance == Robot.Alliance.RED) {
//            if (sample.getColor().equals("Blue")) return Double.NEGATIVE_INFINITY;
//        } else {
//            if (sample.getColor().equals("Red")) return Double.NEGATIVE_INFINITY;
//        }
//
//        // Score based on distance from center
//        double xDist = Math.abs(sample.getX() - center.x);
//        double yDist = Math.abs(sample.getZ() - center.y);
//        double dist = Math.hypot(xDist, yDist);
//
//        double score = 1 / dist + 1;
//
//        // Reduce score if there are other samples in the area in front of this sample (the area closer to the robot).
//        for (Sample otherSample : samples) {
//            if (otherSample == sample) continue;
//            if (otherSample.getZ() < sample.getZ()) {
//                double sampleDist = Math.hypot(otherSample.getX() - sample.getX(), otherSample.getZ() - sample.getZ());
//                score -= 1 / sampleDist;
//            }
//        }
//
//        return score;
//    }

//    private Action CollectSubmersibleSample () {
//        return new SequentialAction(
//                new Intake.MoveSlideActionRR(intake, sampleDistance + SAMPLE_DISTANCE_OFFSET),
//                new Intake.WristActionRR(intake, Intake.WRIST_DOWN_POSITION),
//                new Intake.CollectSampleActionRR(intake)
//        );
//    }
}