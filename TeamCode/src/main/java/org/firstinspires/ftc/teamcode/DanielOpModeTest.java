package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "--DanielOpModeTest")

@Config
public class DanielOpModeTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)).get();
        imu.resetYaw();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        MotorEx frontLeft = new MotorEx(hardwareMap, "leftFront");
        frontLeft.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setRunMode(Motor.RunMode.RawPower);

        MotorEx frontRight = new MotorEx(hardwareMap, "rightFront");
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setInverted(true);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        MotorEx backLeft = new MotorEx(hardwareMap, "leftBack");
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setInverted(true);
        backLeft.setRunMode(Motor.RunMode.RawPower);

        MotorEx backRight = new MotorEx(hardwareMap, "rightBack");
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setInverted(true);
        backRight.setRunMode(Motor.RunMode.RawPower);

        MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);

        while (opModeIsActive()) {
            if (gamepad1.y) {
                imu.resetYaw();
            }

            mecanum.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw());
//            mecanum.driveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


            dashboardTelemetry.update();
        }
    }
}
