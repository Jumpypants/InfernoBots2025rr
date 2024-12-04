package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

    private final Intake intake = new Intake(hardwareMap);
    private final Outtake outtake = new Outtake(hardwareMap);

    private class WristToMid implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setWrist(Intake.WRIST_MID_POSITION);
            return elapsedTime.seconds() > 1.2;
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
            return outtake.stepSlideTo(Outtake.HIGH_BASKET_POSITION, dashboardTelemetry);
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
        Pose2d beginPos = new Pose2d(8.5, 105, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPos);
        intake.getSlideMotor().resetEncoder();
        outtake.getSlideMotor().resetEncoder();

        Actions.runBlocking(new WristToMid());

        TrajectoryActionBuilder goToBucket = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(12, 132, Math.PI / 0.8), Math.PI / 2);

        Actions.runBlocking(new SequentialAction(
                goToBucket.build()
        ));
    }
}
