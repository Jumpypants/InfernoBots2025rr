package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@TeleOp(name = "--DanielOpMode")

public class DanielOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        DriveBase driveBase = new DriveBase(hardwareMap, 1);

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

            driveBase.drive(gamepad1);
        }
    }
}
