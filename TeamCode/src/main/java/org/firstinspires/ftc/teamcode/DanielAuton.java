package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

@Config
@Autonomous(name = "--DanielAuton", group = "Autonomous")
public class DanielAuton extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static double INTAKE_EXTEND_DISTANCE = 10;

    public static double BASKET_X = 4.5;
    public static double BASKET_Y = 20.5;
    public static double BASKET_R = Math.PI * 1.75;

    public static double SAMPLE_1_X = 9.5;
    public static double SAMPLE_1_Y = 8;
    public static double SAMPLE_1_R = 0;

    public static double SAMPLE_2_X = 9.5;
    public static double SAMPLE_2_Y = 22;
    public static double SAMPLE_2_R = 0;

    public static double SAMPLE_3_X = 12;
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

        intake.setWrist(Intake.WRIST_TRANSFER_POSITION);
        intake.setFlip(Intake.FLIP_LOW_POSITION);
        outtake.setSpin(Outtake.SPIN_IN_POSITION);

        while (opModeInInit()) {
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
            sampleFinder.get(dashboardTelemetry);
        }

        Actions.runBlocking(new ParallelAction(
                tickAction,
                mainAction
        ));
    }

    private Action cycleSpikeMark(Pose2d pos) {
        return new SequentialAction(
                new ParallelAction(
                        goTo(pos),
                        retractOuttake(),
                        new Intake.WristActionRR(intake, Intake.WRIST_MID_POSITION),
                        new Intake.FlipActionRR(intake, Intake.FLIP_HIGH_POSITION),
                        new Intake.MoveSlideActionRR(intake, INTAKE_EXTEND_DISTANCE)
                ),
                new Intake.CollectSampleHighActionRR(intake, INTAKE_EXTEND_DISTANCE),
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
                        new Intake.WristActionRR(intake, Intake.WRIST_TRANSFER_POSITION),
                        new Intake.FlipActionRR(intake, Intake.FLIP_LOW_POSITION),
                        new Intake.MoveSlideActionRR(intake, Intake.SLIDE_IN_POSITION)
                ),
                new Intake.TransferSpinActionRR(intake)
        );
    }

    private Action goTo(Pose2d pos) {
        return drive.actionBuilder(beginPos)
                .strafeToLinearHeading(pos.position, pos.heading)
                .build();
    }

    private Action goOuttake () {
        return new SequentialAction(
                new ParallelAction(
                        goTo(basketPos),
                        new SequentialAction(
                                new Intake.ClearWristActionRR(intake),
                                new ParallelAction(
                                        new Outtake.MoveSlideActionRR(outtake, Outtake.HIGH_BASKET_POSITION),
                                        new Outtake.SpinToMidActionRR(outtake)
                                )
                        )
                ),
                new Outtake.DumpActionRR(outtake)
        );
    }
}