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
    @Override
    public void runOpMode() {
        waitForStart();

        DriveBase driveBase = new DriveBase(hardwareMap, 1);

        SampleFinder sampleFinder = new SampleFinder(hardwareMap, telemetry, new double[]{0, 0, 0});

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)).get();

        Outtake outtake = new Outtake(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        TeleOpStateMachine stateMachine = new TeleOpStateMachine();

        while (opModeIsActive()) {
            driveBase.drive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw());

            stateMachine.step(
                    intake,
                    outtake,
                    driveBase,
                    telemetry,
                    imu,
                    gamepad1,
                    gamepad2,
                    sampleFinder
            );

            dashboardTelemetry.update();
        }
    }
}
