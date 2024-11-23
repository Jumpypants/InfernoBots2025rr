package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "--Test")

@Config
public class DanielOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                intake.setWrist(Intake.WRIST_DOWN_POSITION);
            }
        }
    }
}
