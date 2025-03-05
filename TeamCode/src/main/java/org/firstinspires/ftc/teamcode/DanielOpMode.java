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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.murphy.MurphyStateMachine;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.robotStates.OuttakingState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

@TeleOp(name = "--DanielOpMode")

@Config
public class DanielOpMode extends LinearOpMode {
    MotorEx backRight;
    MotorEx frontRight;
    MotorEx backLeft;
    MotorEx frontLeft;

    public static double KP = 0.001;
    public static double KI = 0.0;
    public static double KD = 0.004;

    public static double ROTATION_ASSIST = 0.01;
    private final PIDController ROTATION_PID = new PIDController(KP, KI, KD);

    @Override
    public void runOpMode() {
        MecanumDrive driveBase = getDriveBase();

        SampleFinder sampleFinder = new SampleFinder(hardwareMap, telemetry, new double[]{0, 0, 0});

        IMU imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)
        ).get();
        //imu.resetYaw();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Kicker kicker = new Kicker(hardwareMap);

        outtake.setSpin(Outtake.SPIN_IN_POSITION);
        intake.setWrist(Intake.WRIST_TRANSFER_POSITION);
        outtake.setClaw(Outtake.CLAW_CLOSED_POSITION);
        intake.setFlip(Intake.FLIP_LOW_POSITION);
        kicker.setKicker(Kicker.IN_POSITION);

        Robot robot = new Robot(
                gamepad1,
                gamepad2,
                intake,
                outtake,
                driveBase,
                imu,
                dashboardTelemetry,
                Robot.Alliance.RED,
                kicker
        );

        while (opModeInInit()) {
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

            intake.setWrist(Intake.WRIST_TRANSFER_POSITION);
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


            stateMachine.step(dashboardTelemetry);

            intake.stepSlide(dashboardTelemetry);
            outtake.stepSlide(dashboardTelemetry);

            stepDriveBase(driveBase, imu, robot);

            dashboardTelemetry.addData("FrontLeft", frontLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("FrontRight", frontRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("BackLeft", backLeft.motorEx.getCurrent(CurrentUnit.MILLIAMPS));
            dashboardTelemetry.addData("BackRight", backRight.motorEx.getCurrent(CurrentUnit.MILLIAMPS));

            dashboardTelemetry.update();
        }
    }

    private void stepDriveBase(MecanumDrive driveBase, IMU imu, Robot robot) {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        int target = 135;
        double yaw = imu.getRobotYawPitchRollAngles().getYaw();
        if (gamepad1.left_bumper && Math.abs(target - yaw) > 20) {
            ROTATION_PID.setSetpoint(getAngleWithSign(target, yaw));
            r = ROTATION_PID.calculate(yaw);
        }

        if (gamepad1.left_trigger > 0.1) {
            x /= 2;
            y /= 2;
            r /= 2;
        } else if (gamepad1.right_trigger > 0.1) {
            x /= 4;
            y /= 4;
            r /= 4;
        }

        //r += robot.rotationOffset;

        driveBase.driveFieldCentric(x, y, r, imu.getRobotYawPitchRollAngles().getYaw());
    }

    private double getAngleWithSign(double target, double current) {
        // Will return the target angle in either the positive or negative direction, whichever is shorter.
        double negative = target - 360;

        if (Math.abs(negative - current) < Math.abs(target - current)) {
            return negative;
        } else {
            return target;
        }
    }

    private MecanumDrive getDriveBase() {
        frontLeft = new MotorEx(hardwareMap, "leftFront");
        frontLeft.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setRunMode(Motor.RunMode.RawPower);

        frontRight = new MotorEx(hardwareMap, "rightFront");
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setInverted(true);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        backLeft = new MotorEx(hardwareMap, "leftBack");
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setInverted(true);
        backLeft.setRunMode(Motor.RunMode.RawPower);

        backRight = new MotorEx(hardwareMap, "rightBack");
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setInverted(true);
        backRight.setRunMode(Motor.RunMode.RawPower);

        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
}
