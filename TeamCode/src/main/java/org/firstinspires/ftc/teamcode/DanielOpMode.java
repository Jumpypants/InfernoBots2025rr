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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.murphy.MurphyStateMachine;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    public static double KP = 0.2;
    public static double KI = 0.0;
    public static double KD = 0.01;
    private final PIDController ROTATION_PID = new PIDController(KP, KI, KD);

    @Override
    public void runOpMode() {
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

        outtake.setSpin(Outtake.SPIN_IN_POSITION);
        intake.setWrist(Intake.WRIST_UP_POSITION);

        Robot robot = new Robot(
                gamepad1,
                gamepad2,
                intake,
                outtake,
                driveBase,
                imu,
                dashboardTelemetry,
                Robot.Alliance.RED
        );

        while (!opModeIsActive()) {
            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
            }
            if (gamepad2.dpad_down) {
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad2.dpad_right) {
                robot.alliance = Robot.Alliance.RED;
            }
            if (gamepad2.dpad_left) {
                robot.alliance = Robot.Alliance.BLUE;
            }
        }

        MurphyStateMachine stateMachine = new MurphyStateMachine(new IntakingState(robot));

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                intake.getSlideMotor().resetEncoder();
            }
            if (gamepad2.dpad_down) {
                outtake.getSlideMotor().resetEncoder();
            }

            if (gamepad2.dpad_right) {
                robot.alliance = Robot.Alliance.RED;
            }
            if (gamepad2.dpad_left) {
                robot.alliance = Robot.Alliance.BLUE;
            }

            if (gamepad1.y) {
                imu.resetYaw();
            }

            if (gamepad1.a) {
                outtake.setSpin(Outtake.SPIN_IN_POSITION);
            }
            if (gamepad1.b) {
                outtake.setSpin(Outtake.SPIN_MID_POSITION);
            }
            if (gamepad1.x) {
                outtake.setSpin(Outtake.SPIN_OUT_POSITION);
            }


            stateMachine.step(dashboardTelemetry);

            intake.stepSlide(dashboardTelemetry);
            outtake.stepSlide(dashboardTelemetry);

            stepDriveBase(driveBase, imu);

            dashboardTelemetry.update();
        }
    }

    private void stepDriveBase(MecanumDrive driveBase, IMU imu) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        if (gamepad2.right_bumper) {
            int target = 90;
            ROTATION_PID.setSetpoint(getAngleWithSine(target, imu.getRobotYawPitchRollAngles().getYaw()));
            r = ROTATION_PID.calculate(imu.getRobotYawPitchRollAngles().getYaw());
        } else if (gamepad2.left_bumper) {
            int target = 270;
            ROTATION_PID.setSetpoint(getAngleWithSine(target, imu.getRobotYawPitchRollAngles().getYaw()));
            r = ROTATION_PID.calculate(imu.getRobotYawPitchRollAngles().getYaw());
        }

        if (gamepad1.left_trigger > 0.1) {
            x /= 2;
            y /= 2;
            r /= 2;
        }

        driveBase.driveFieldCentric(x, y, r, imu.getRobotYawPitchRollAngles().getYaw());
    }

    private double getAngleWithSine(double target, double current) {
        // Will return the target angle in either the positive or negative direction, whichever is shorter.
        double negative = target - 360;

        if (Math.abs(negative - current) < Math.abs(target - current)) {
            return negative;
        } else {
            return target;
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
