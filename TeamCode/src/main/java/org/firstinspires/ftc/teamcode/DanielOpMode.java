package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.murphy.MurphyStateMachine;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive driveBase = getDriveBase();

        SampleFinder sampleFinder = new SampleFinder(hardwareMap, telemetry, new double[]{0, 0, 0});

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        ).get();
        imu.resetYaw();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        Robot robot = new Robot(
                gamepad1,
                gamepad2,
                intake,
                outtake,
                driveBase,
                imu
        );

        MurphyStateMachine stateMachine = new MurphyStateMachine(new IntakingState(robot));

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
            }
            if (gamepad2.dpad_down) {
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad2.dpad_right) {
                outtake.getSlideMotor().set(-0.5);
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad1.y) {
                imu.resetYaw();
            }

            stateMachine.step(dashboardTelemetry);

            intake.stepSlide();
            outtake.stepSlide();

            stepDriveBase(driveBase, imu);

            dashboardTelemetry.update();
        }
    }

    private void stepDriveBase(MecanumDrive driveBase, IMU imu) {
        if (gamepad1.left_trigger > 0.1) {
            driveBase.driveFieldCentric(gamepad1.left_stick_x / 2, -gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2, imu.getRobotYawPitchRollAngles().getYaw());
        } else {
            driveBase.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw());
        }
    }

    private MecanumDrive getDriveBase() {
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

        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
}
