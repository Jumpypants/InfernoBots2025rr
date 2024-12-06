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
    public static double targetPos = 0;

    @Override
    public void runOpMode() {
        waitForStart();

        DriveBase driveBase = new DriveBase(hardwareMap);

        SampleFinder sampleFinder = new SampleFinder(hardwareMap, telemetry, new double[]{0, 0, 0});

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)).get();
        imu.resetYaw();

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        TeleOpStateMachine stateMachine = new TeleOpStateMachine();

        intake.getSlideMotor().resetEncoder();
        outtake.getSlideMotor().resetEncoder();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
            }
            if (gamepad2.dpad_down) {
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad1.y) {
                imu.resetYaw();
            }
            driveBase.drive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw(), dashboardTelemetry);
            //intake.setWrist(Intake.WRIST_DOWN_POSITION);
            stateMachine.step(
                    intake,
                    outtake,
                    driveBase,
                    dashboardTelemetry,
                    imu,
                    gamepad1,
                    gamepad2,
                    sampleFinder
            );

            dashboardTelemetry.update();
        }
    }
}
