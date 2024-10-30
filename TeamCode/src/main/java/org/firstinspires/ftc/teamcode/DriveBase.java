package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Config
public class DriveBase {
    ArrayList<Motor> motors;
    public static double velocityConst;
    public static double rotationConst = 0.0645;
    public static double rotationKp = 0.0175;
    public static double rotationKd = 0;

    public DriveBase (HardwareMap hardwareMap, double velocityConst) {
        this.motors = findMotors(hardwareMap);
        DriveBase.velocityConst = velocityConst;

        for (Motor motor : this.motors) {
            motor.setRunMode(Motor.RunMode.VelocityControl);
            motor.setVeloCoefficients(0.05, 0, 0);
        }
    }

    private ArrayList<Motor> findMotors(HardwareMap hardwareMap) {
        Motor frontLeft = new Motor(hardwareMap, "leftFront");
        frontLeft.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        Motor frontRight = new Motor(hardwareMap, "rightFront");
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        Motor backLeft = new Motor(hardwareMap, "leftBack");
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setInverted(true);
        Motor backRight = new Motor(hardwareMap, "rightBack");
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        ArrayList<Motor> driveMotors = new ArrayList<Motor>();
        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        return driveMotors;
    }

    public void drive (Gamepad gamepad1, double heading) {
        resetMotorPowers();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        // Rotate the joystick vector by the heading for field-centric driving
        double cosHeading = Math.cos(Math.toRadians(heading));
        double sinHeading = Math.sin(Math.toRadians(heading));

        double strafe = x * cosHeading - y * sinHeading;
        double drive = -(x * sinHeading + y * cosHeading);
        double turn = gamepad1.right_stick_x;

        double leftFrontPower = drive + turn + strafe;
        double rightFrontPower = drive - turn - strafe;
        double leftBackPower = drive + turn - strafe;
        double rightBackPower = drive - turn + strafe;

        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        motors.get(0).set(leftFrontPower * velocityConst);
        motors.get(1).set(rightFrontPower * velocityConst);
        motors.get(2).set(leftBackPower * velocityConst);
        motors.get(3).set(rightBackPower * velocityConst);
    }

    public void driveRotateTo (double target, Telemetry telemetry, double kpMultiplier) {
        telemetry.addData("rotationConst", Double.toString(DriveBase.rotationConst));
        telemetry.addData("rotationKp", Double.toString(DriveBase.rotationKp * kpMultiplier));

        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

        PIDController pidController = new PIDController(DriveBase.rotationKp * kpMultiplier, 0, rotationKd);
        pidController.setSetpoint(target);

        double power;

        if (pidController.calculate(0) < 0) {
            power = rotationConst - pidController.calculate(0);
        } else {
            power = -rotationConst - pidController.calculate(0);
        }

        leftFrontDrive.set(-power);
        rightFrontDrive.set(power);
        leftBackDrive.set(-power);
        rightBackDrive.set(power);
    }

    public void resetMotorPowers () {
        for (Motor m : motors) {
            m.set(0);
        }
    }
}
