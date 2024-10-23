package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    public static double targetPos = 0;

    @Override
    public void runOpMode() {
        waitForStart();

        DriveBase driveBase = new DriveBase(hardwareMap, 1);
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

//            driveBase.drive(gamepad1);

            dashboardTelemetry.addData("slidePosition in rotations", intake.getSLIDE_MOTOR().getCurrentPosition() / 360);
            dashboardTelemetry.addData("slidePostion in inches", intake.getSlidePosition());

            intake.setSpin(0);
            if (gamepad1.a) {
                intake.setSpin(1);
            }

            if (gamepad1.b) {
                intake.setSpin(-1);
            }

            if (gamepad1.dpad_up) {
               intake.setWrist(200);
            }

            if (gamepad1.dpad_down) {
                intake.setWrist(0);
            }

            if (gamepad1.x) {
                targetPos = 0;
            }

            if (gamepad1.y) {
                targetPos = 12;
            }


            //outtake.stepSlideTo(targetPos, dashboardTelemetry);
            intake.stepSlideTo(targetPos);
            dashboardTelemetry.update();
        }
    }
}
