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
        Motor frontRight = new Motor(hardwareMap, "rightFront");
        frontRight.setInverted(true);
        Motor backLeft = new Motor(hardwareMap, "leftBack");
        Motor backRight = new Motor(hardwareMap, "rightBack");

        ArrayList<Motor> driveMotors = new ArrayList<Motor>();
        driveMotors.add(frontLeft);
        driveMotors.add(frontRight);
        driveMotors.add(backLeft);
        driveMotors.add(backRight);

        return driveMotors;
    }

    public void drive (Gamepad gamepad1) {
//        telemetry.addData("joystick y", gamepad1.left_stick_y);
//        telemetry.update();

        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
        double lateral = -gamepad1.left_stick_y;
        double yaw     = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        leftFrontPower  *= velocityConst;
        rightFrontPower *= velocityConst;
        leftBackPower   *= velocityConst;
        rightBackPower  *= velocityConst;

        // Send calculated power to wheels
        leftFrontDrive.set(leftFrontPower);
        rightFrontDrive.set(rightFrontPower);
        leftBackDrive.set(leftBackPower);
        rightBackDrive.set(rightBackPower);
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
