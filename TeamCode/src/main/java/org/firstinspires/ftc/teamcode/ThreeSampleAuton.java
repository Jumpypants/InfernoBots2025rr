package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "--ThreeSampleAuton", group = "Autonomous")
public class ThreeSampleAuton extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Intake intake;
    private Outtake outtake;

    private class WristToMid implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setWrist(Intake.WRIST_MID_POSITION);
            return elapsedTime.seconds() > 1;
        }
    }

    private class ExtendToSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double SAMPLE_DIST = 5;
            return intake.stepSlideTo(SAMPLE_DIST, dashboardTelemetry);
        }
    }

    private class WristToSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setWrist(Intake.WRIST_DOWN_POSITION);
            return elapsedTime.seconds() > 1.2;
        }
    }

    private class ExtendToOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return outtake.stepSlideTo(Outtake.HIGH_BASKET_POSITION, telemetryPacket);
        }
    }

    private class DumpSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.setSpin(Outtake.SPIN_OUT_POSITION);
            return elapsedTime.seconds() > 1.2;
        }
    }

    private class RetractOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (outtake.stepSlideTo(Outtake.DOWN_POSITION, dashboardTelemetry)) {
                outtake.getSlideMotor().set(0);
                return true;
            }
            return false;
        }
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.getSlideMotor().resetEncoder();
        outtake.getSlideMotor().resetEncoder();

        Pose2d beginPos = new Pose2d(-14, -60, Math.PI / 2);
        Pose2d basketPos = new Pose2d(-57, -57, Math.PI * 0.75);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPos);

        TrajectoryActionBuilder goToBucket = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(59, 59, Math.PI * 0.75), Math.PI * 1.5);

        TrajectoryActionBuilder goToSample1 = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(35, 38, Math.PI * 1.75), Math.PI / 2);

        TrajectoryActionBuilder goToSample2 = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(44.5, 38, Math.PI * 1.75), Math.PI / 2);

        TrajectoryActionBuilder goToSample3 = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(54, 38, Math.PI * 1.75), Math.PI / 2);

        SequentialAction goOuttake = new SequentialAction(
                new ParallelAction(
                    goToBucket.build(),
                    new SequentialAction(
                            new WristToMid(),
                            new ExtendToOuttake()
                    )
                ),
                new DumpSample()
        );

        while (!opModeIsActive());

        Actions.runBlocking(new ExtendToOuttake());

        while (opModeIsActive()) {
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    private Pose2d getPositionForSample (double x, double y) {
        return new Pose2d(x, y, Math.PI * 1.75);
    }
}
