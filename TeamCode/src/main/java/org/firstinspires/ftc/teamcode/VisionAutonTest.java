package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

import java.util.ArrayList;
import java.util.Comparator;

@Config
@Autonomous(name = "VisionAutonTest", group = "Autonomous")
public class VisionAutonTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Robot.Alliance alliance = Robot.Alliance.BLUE;
    private SampleFinder sampleFinder;
    private MecanumDrive drive;

    private Intake intake;
    private Kicker kicker;

    private Sample selectedSample;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        sampleFinder = new SampleFinder(hardwareMap, dashboardTelemetry, new double[]{0, 0, 0});

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        intake = new Intake(hardwareMap);
        intake.getSlideMotor().resetEncoder();
        intake.setWrist(Intake.WRIST_UP_POSITION);
        intake.setFlip(Intake.FLIP_HIGH_POSITION);

        kicker = new Kicker(hardwareMap);
        kicker.setKicker(Kicker.IN_POSITION);

        while (opModeInInit()) {
            dashboardTelemetry.update();
        }

        Action tickAction = telemetryPacket -> {
            intake.stepSlide(dashboardTelemetry);

            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
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

        Action mainAction = new SequentialAction(
                new Kicker.KickActionRR(kicker),
                waitAction(0.2),
                findAction(),
                new ParallelAction(
                        new RotateAction(),
                        new Intake.WristActionRR(intake, Intake.WRIST_DOWN_POSITION),
                        new Intake.FlipActionRR(intake, Intake.FLIP_LOW_POSITION)
                ),
                new Intake.CollectSampleLowActionRR(intake, alliance)
        );

        Actions.runBlocking(new ParallelAction(
                mainAction,
                tickAction
        ));

        while (opModeIsActive()) {
            dashboardTelemetry.update();
        }
    }

    private Action findAction() {
        return telemetryPacket -> {
            ArrayList<Sample> samples = sampleFinder.get(telemetry);

            dashboardTelemetry.addData("sample count", samples.size());

            // Remove samples that have another sample in an area in front of them
            samples.removeIf(sample -> {
                for (Sample otherSample : samples) {
                    if (sample != otherSample) {
                        if (otherSample.getX() > sample.getX() - 1.5
                                && otherSample.getX() < sample.getX() + 1.5
                                && otherSample.getY() < sample.getY()) {
                            return true;
                        }
                    }
                }

                return false;
            });

            samples.removeIf(sample -> sample.getColor().equals("Red"));

            if (samples.isEmpty()) return true;

            // Sort the samples by how close they are to x = 0
            samples.sort(Comparator.comparingDouble(sample -> Math.abs(sample.getX())));

            selectedSample = samples.get(0);

            dashboardTelemetry.addData("selected Sample", selectedSample.toString());
            dashboardTelemetry.addData("selected Sample angle", selectedSample.getAngle());
            dashboardTelemetry.addData("selected Sample distance", selectedSample.getDistance());

            return false;
        };
    }

    private Action waitAction(double t) {
        return new Action() {
            private final ElapsedTime elapsedTime = new ElapsedTime();
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    elapsedTime.reset();
                }

                return elapsedTime.seconds() < t;
            }
        };
    }

    private class RotateAction implements Action {
        Action driveAction = null;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (driveAction == null) {
                driveAction = drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(selectedSample.getAngle()))
                        .build();
            }

            return driveAction.run(telemetryPacket);
        }
    }

    private class Extend implements Action {
        Action extendAction = null;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (extendAction == null) {
                extendAction = new Intake.MoveSlideActionRR(intake, selectedSample.getDistance());
            }

            return extendAction.run(telemetryPacket);
        }
    }
}