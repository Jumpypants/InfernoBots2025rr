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
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

import java.util.ArrayList;

@Config
@Autonomous(name = "VisionAutonTest", group = "Autonomous")
public class VisionAutonTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Robot.Alliance alliance = Robot.Alliance.RED;
    private SampleFinder sampleFinder;
    private MecanumDrive drive;

    private Intake intake;

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
                findAction(),
//                new Rotate(),
//                waitAction(1),
//                findAction(),
                new ParallelAction(
                        new Rotate(),
                        new Extend()
                ),
                new Intake.WristActionRR(intake, Intake.WRIST_MID_POSITION),
                new Intake.CollectSampleHighActionRR(intake)
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

            samples.removeIf(sample -> !sample.getColor().equals("Blue"));

            dashboardTelemetry.addData("sample count", samples.size());

            if (samples.isEmpty()) return true;

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

    private class Rotate implements Action {
        Action driveAction = null;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (driveAction == null) {
                driveAction = drive.actionBuilder(drive.pose)
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