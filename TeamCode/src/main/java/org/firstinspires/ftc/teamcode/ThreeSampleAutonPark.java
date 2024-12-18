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

import java.util.ArrayList;
import java.util.Objects;

@Autonomous(name = "--ThreeSampleAutonPark", group = "Autonomous")
public class ThreeSampleAutonPark extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Intake intake;
    private Outtake outtake;

    private SampleFinder sampleFinder;

    private double sampleAngle = 0;

    private class WristToMid implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "WristToMid");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            intake.setWrist(Intake.WRIST_MID_POSITION);
            return elapsedTime.seconds() < 0.3;
        }
    }

    private class FindSampleAngle implements Action {
        private boolean initialized = false;

        private ArrayList<Sample> samples;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "RotateToSample");
            dashboardTelemetry.update();
            if (!initialized) {
                initialized = true;
                samples = sampleFinder.get(dashboardTelemetry);
            }

            samples.removeIf(sample -> !Objects.equals(sample.color, "Yellow"));

            if (samples.isEmpty()) return false;

            sampleAngle = samples.get(0).getAngle();

            return false;
        }
    }

    private class WristToUp implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "WristToUp");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            intake.setWrist(Intake.WRIST_UP_POSITION);
            return elapsedTime.seconds() < 1.1;
        }

}
    private class CollectSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "CollectSample");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
                intake.setSpin(Intake.SPIN_IN);
            }

            if (!intake.stepSlideTo(13.5, 0.55, dashboardTelemetry) || elapsedTime.seconds() < 1.4) {
                return true;
            }
            intake.setSpin(Intake.SPIN_STOP);
            return false;
        }
    }

    private class ExtendToSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "ExtendToSample");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            double SAMPLE_DIST = 3;
            if (elapsedTime.seconds() < 1.2) {
                return true;
            }
            return !intake.stepSlideTo(SAMPLE_DIST, dashboardTelemetry);
        }
    }

    private class RetractIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "RetractIntake");
            dashboardTelemetry.update();
            return !intake.stepSlideTo(Intake.SLIDE_IN_POSITION, dashboardTelemetry);
        }
    }

    private class WristToSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "WristToSample");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            intake.setWrist(Intake.WRIST_DOWN_POSITION);
            return elapsedTime.seconds() <= 1.2;
        }
    }

    private class OuttakeSpinToIn implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "OuttakeSpinToIn");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            outtake.setSpin(Outtake.SPIN_IN_POSITION);
            return elapsedTime.seconds() <= 1.2;
        }
    }

    private class Transfer implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "Transfer");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                intake.setSpin(Intake.SPIN_OUT);
                initialized = true;
            }

            if (elapsedTime.seconds() > Intake.TRANSFER_SPIN_TIME) {
                intake.setSpin(Intake.SPIN_STOP);
                return false;
            }
            return true;
        }
    }

    private class ExtendToOuttake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "ExtendToOuttake");
            dashboardTelemetry.update();
            return !outtake.stepSlideTo(Outtake.HIGH_BASKET_POSITION, telemetryPacket);
        }
    }

    private class RetractOuttake implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "RetractOuttake");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            if (elapsedTime.seconds() < 1) return true;
            if (outtake.stepSlideTo(Outtake.DOWN_POSITION, telemetryPacket)) {
                outtake.setSlidePower(0);
                outtake.getSlideMotor().resetEncoder();
                return false;
            }
            return true;
        }
    }

    private class DumpSample implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            dashboardTelemetry.addData("Action", "DumpSample");
            dashboardTelemetry.update();
            if (!initialized) {
                elapsedTime.reset();
                initialized = true;
            }
            outtake.setSpin(Outtake.SPIN_OUT_POSITION);
            return elapsedTime.seconds() <= Outtake.TIME_TO_SPIN;
        }
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        //sampleFinder = new SampleFinder(hardwareMap, dashboardTelemetry, new double[]{0, 0, 0});


        intake.getSlideMotor().resetEncoder();
        outtake.getSlideMotor().resetEncoder();

        Pose2d beginPos = new Pose2d(-38, -60, Math.PI * 1.5);
        Pose2d basketPos = new Pose2d(-58.5, -53.25, Math.PI * 1.75);
        Pose2d endPos = new Pose2d(55, -50, Math.PI * 0.5);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPos);

        TrajectoryActionBuilder goToBasket = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(basketPos, Math.PI * 1.2);
        TrajectoryActionBuilder goToEndPos = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(endPos, Math.PI * 1.2);


        TrajectoryActionBuilder goToSample1 = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-36.5, -31.5, Math.PI * 0.75), Math.PI / 2);

        TrajectoryActionBuilder goToSample2 = drive.actionBuilder(beginPos)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-47, -30.5, Math.PI * 0.75), Math.PI / 2);

//        TrajectoryActionBuilder goToSample3 = drive.actionBuilder(beginPos)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(-58.5, -30, Math.PI * 0.75), Math.PI * 1.5);


        waitForStart();

        intake.setWrist(Intake.WRIST_UP_POSITION);
        outtake.setSpin(Outtake.SPIN_IN_POSITION);

        Actions.runBlocking(new SequentialAction(
                goOuttake(goToBasket),
                grabSample(goToSample1.build()),
                goOuttake(goToBasket),
                grabSample(goToSample2.build()),
                goOuttake(goToBasket),
//                grabSample(goToSample3.build()),
//                goOuttake(goToBasket)
                new ParallelAction(
                        new RetractOuttake(),
                        new OuttakeSpinToIn(),
                        goToEndPos.build()
                )
        ));

        while (opModeIsActive()) {
            dashboardTelemetry.update();
            telemetry.update();
        }
    }

    private Pose2d getPositionForSample (double x, double y) {
        return new Pose2d(x, y, Math.PI * 1.75);
    }

    private Action goOuttake (TrajectoryActionBuilder goToBasket) {
        return new SequentialAction(
                new ParallelAction(
                        goToBasket.build(),
                        new SequentialAction(
                            new WristToMid(),
                            new ExtendToOuttake()
                        )
                ),
                new DumpSample()
        );
    }

    private Action transfer () {
        return new SequentialAction(
                new ParallelAction(
                        new RetractIntake(),
                        new WristToUp()
                ),
                new Transfer(),
                new WristToMid()
        );
    }

    private Action grabSample (Action trajectory) {
        return new SequentialAction(
                new ParallelAction(
                        trajectory,
                        new RetractOuttake(),
                        new ExtendToSample(),
                        new WristToSample(),
                        new OuttakeSpinToIn()
                ),
                new CollectSample(),
                transfer()
        );
    }
}
