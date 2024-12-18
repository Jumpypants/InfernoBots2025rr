package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "--IntakeTest")

@Config
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        CRServo left = new CRServo(hardwareMap, "left");
        left.setInverted(true);
        CRServo right = new CRServo(hardwareMap, "right");

        left.set(1);
        right.set(1);
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                left.set(-left.get());
                right.set(-right.get());
            }
        }
    }
}
