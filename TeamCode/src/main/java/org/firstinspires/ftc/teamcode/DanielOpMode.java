package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    public static double targetPos = 0;
    public static double targetWristPos = 0.82;

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
//            driveBase.resetMotorPowers();
//
//            ArrayList<Sample> samples = sampleFinder.getDetectedStonePositions(dashboardTelemetry);
//            if (!samples.isEmpty()) {
//                driveBase.driveRotateTo(samples.get(0).x, dashboardTelemetry, 1 / samples.get(0).z * 100);
//            }
//            dashboardTelemetry.update();

            driveBase.drive(gamepad1, imu.getRobotYawPitchRollAngles().getYaw());
            //driveBase.drive(gamepad1, 0);

//            dashboardTelemetry.addData("slidePosition in rotations", intake.getSLIDE_MOTOR().getCurrentPosition() / 360);
//            dashboardTelemetry.addData("slidePosition in inches", intake.getSlidePosition());

//            intake.setSpin(0);
//            if (gamepad1.a) {
//                intake.setSpin(1);
//            }
//
//            if (gamepad1.b) {
//                intake.setSpin(-1);
//            }
//
//            if (gamepad1.dpad_up) {
//               targetWristPos = 0.225;
//            }
//
//            if (gamepad1.dpad_down) {
//                targetWristPos = 0.82;
//            }
//
//            if (gamepad1.x) {
//                targetPos = 0;
//            }
//
//            if (gamepad1.y) {
//                targetPos = 12;
//            }

            //dashboardTelemetry.addData("Wrist target", targetWristPos);


            //outtake.stepSlideTo(targetPos, dashboardTelemetry);
            //intake.stepSlideTo(targetPos);
            //intake.setWrist(targetWristPos);
            dashboardTelemetry.update();
        }
    }
}
