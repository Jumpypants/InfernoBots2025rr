package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.ArrayList;

@Config
public class DriveBase {
    ArrayList<Motor> motors;
    // TODO: re-tune PID constants
    public static double ROTATION_CONST = 0.0645;
    public static double ROTATION_KP = 0.0175;
    public static double ROTATION_KD = 0;
    public static double ROTATION_KI = 0;
    public static double ROTATION_ALLOWED_ERROR = 0.1;

    public DriveBase (HardwareMap hardwareMap) {
        this.motors = findMotors(hardwareMap);

        for (Motor motor : this.motors) {
            motor.setRunMode(Motor.RunMode.RawPower);
//            motor.setVeloCoefficients(0.05, 0, 0);
        }
    }

    private ArrayList<Motor> findMotors(HardwareMap hardwareMap) {
        Motor frontLeft = new Motor(hardwareMap, "leftFront");
        frontLeft.setInverted(true);
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setRunMode(Motor.RunMode.RawPower);

        Motor frontRight = new Motor(hardwareMap, "rightFront");
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        Motor backLeft = new Motor(hardwareMap, "leftBack");
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setInverted(true);
        backLeft.setRunMode(Motor.RunMode.RawPower);

        Motor backRight = new Motor(hardwareMap, "rightBack");
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setRunMode(Motor.RunMode.RawPower);

        ArrayList<Motor> driveMotors = new ArrayList<Motor>();
        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        return driveMotors;
    }

    public void drive (Gamepad gamepad1, double heading, Telemetry telemetry) {
        resetMotorPowers();
        
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        // Rotate the joystick vector by the heading for field-centric driving
        double cosHeading = Math.cos(Math.toRadians(heading));
        double sinHeading = Math.sin(Math.toRadians(heading));

        double strafe = x * cosHeading - y * sinHeading;
        double drive = -(x * sinHeading + y * cosHeading);
        double turn = 0;

        telemetry.addData("heading", heading);
        if (gamepad1.right_bumper) {
            double BASKET_ANGLE = 45;
            stepRotateTo(BASKET_ANGLE, heading, telemetry, 1);
        } else {
            turn = gamepad1.right_stick_x;
        }

        double leftFrontPower = drive + turn + strafe;
        double rightFrontPower = drive - turn - strafe;
        double leftBackPower = drive + turn - strafe;
        double rightBackPower = drive - turn + strafe;

        //double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        double maxPower = 1;
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        if (gamepad1.left_trigger > 0) {
            leftFrontPower *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower *= 0.5;
            rightBackPower *= 0.5;
        }
        
        if (gamepad1.right_trigger > 0) {
            leftFrontPower *= 0.2;
            rightFrontPower *= 0.2;
            leftBackPower *= 0.2;
            rightBackPower *= 0.2;
        }

        motors.get(0).set(leftFrontPower);
        motors.get(1).set(rightFrontPower);
        motors.get(2).set(leftBackPower );
        motors.get(3).set(rightBackPower);
    }

    public double strafe (Gamepad gamepad1, double heading, Telemetry telemetry) {
        resetMotorPowers();

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        // Rotate the joystick vector by the heading for field-centric driving
        double cosHeading = Math.cos(Math.toRadians(heading));
        double sinHeading = Math.sin(Math.toRadians(heading));

        double strafe = x * cosHeading - y * sinHeading;
        double drive = -(x * sinHeading + y * cosHeading);

        double leftFrontPower = strafe;
        double rightFrontPower = -strafe;
        double leftBackPower = -strafe;
        double rightBackPower = strafe;

        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        if (gamepad1.left_trigger > 0) {
            leftFrontPower *= 0.5;
            rightFrontPower *= 0.5;
            leftBackPower *= 0.5;
            rightBackPower *= 0.5;
        }

        if (gamepad1.right_trigger > 0) {
            leftFrontPower *= 0.2;
            rightFrontPower *= 0.2;
            leftBackPower *= 0.2;
            rightBackPower *= 0.2;
        }

        motors.get(0).set(leftFrontPower);
        motors.get(1).set(rightFrontPower);
        motors.get(2).set(leftBackPower );
        motors.get(3).set(rightBackPower);

        return drive;
    }

    public boolean stepRotateTo(double target, double heading, Telemetry telemetry, double kpMultiplier) {
        // Normalize the target heading relative to the current heading
        double rawDifference = target - heading;
        double normalizedTarget = heading + ((rawDifference + 180) % 360 - 180);

        // Check if the robot is within the allowed error of the target
        if (Math.abs(normalizedTarget - heading) < ROTATION_ALLOWED_ERROR) {
            resetMotorPowers();
            return true;
        }

        // Get motor instances
        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

        // Create the PID controller with the adjusted target
        PIDController pidController = new PIDController(DriveBase.ROTATION_KP * kpMultiplier, ROTATION_KI, ROTATION_KD);
        pidController.setSetpoint(normalizedTarget);

        // Calculate power using the PID controller
        double pidOutput = pidController.calculate(heading);
        double power;

        if (pidOutput < 0) {
            power = ROTATION_CONST - pidOutput;
        } else {
            power = -ROTATION_CONST - pidOutput;
        }

        // Set motor powers to rotate the robot
        leftFrontDrive.set(-power);
        rightFrontDrive.set(power);
        leftBackDrive.set(-power);
        rightBackDrive.set(power);

        return false;
    }

    public void rotate(double power) {
        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

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

    public void moveSideWays(int power) {
        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

        leftFrontDrive.set(power);
        rightFrontDrive.set(-power);
        leftBackDrive.set(-power);
        rightBackDrive.set(power);
    }
}
