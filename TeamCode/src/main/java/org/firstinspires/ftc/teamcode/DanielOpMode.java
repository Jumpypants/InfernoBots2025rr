package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    private static enum State {
        Transfer,
        ExtendVertical,
        ExtendHorizontal,
        None
    }

    private State state = State.None;

    @Override
    public void runOpMode() {
        waitForStart();

        DriveBase driveBase = new DriveBase(hardwareMap, 1);

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)).get();

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        //SampleFinder sampleFinder = new SampleFinder(hardwareMap, dashboardTelemetry);

        while (opModeIsActive()) {
            driveBase.drive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw());

//            dashboardTelemetry.addData("slidePosition in rotations", intake.getSLIDE_MOTOR().getCurrentPosition() / 360);
//            dashboardTelemetry.addData("slidePosition in inches", intake.getSlidePosition());

            if (gamepad1.a) {
                intake.setState(Intake.State.EnterSubmersible);
                state = State.ExtendHorizontal;
            }

            if (gamepad1.b) {
                intake.setState(Intake.State.Transfer);
                state = State.Transfer;
            }

            switch (state) {
                case Transfer:
                    if (intake.finishedTransfer) {
                        state = State.ExtendVertical;
                        outtake.setState(Outtake.State.OuttakeHigh);
                    }
                    break;
                case ExtendVertical:
                    if (outtake.finishedOuttake) {
                        state = State.None;
                        outtake.setState(Outtake.State.GoDown);
                    }
                    break;
                case ExtendHorizontal:
                    if (intake.finishedIntake) {
                        state = State.Transfer;
                    }
                    break;
            }

            intake.step();
            outtake.step(telemetry);

            dashboardTelemetry.update();
        }
    }
}
