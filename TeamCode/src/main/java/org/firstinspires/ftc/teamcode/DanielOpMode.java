package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "--DanielOpMode")

public class DanielOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        Gamepad gamepad1 = new Gamepad();

        DriveBase driveBase = new DriveBase(hardwareMap, 1);
        Outtake outtake = new Outtake(hardwareMap);

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

            dashboardTelemetry.addData("a", "not pressed");

            if (gamepad1.a) {
                outtake.stepTurnOut();
                telemetry.addData("a", "Pressed");
            }

            if (gamepad1.b) {
                outtake.stepTurnIn();
            }

            if (gamepad1.x) {
                outtake.stepSlideTo(0);
            }

            if (gamepad1.y) {
                outtake.stepSlideTo(10);
            }

            dashboardTelemetry.update();
        }
    }
}
