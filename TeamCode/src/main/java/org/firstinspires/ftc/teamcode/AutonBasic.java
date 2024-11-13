package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

@Autonomous(name = "--AutonBasic")
public class AutonBasic extends LinearOpMode {
    ArrayList<Motor> motors;
    int c = 0;

    @Override
    public void runOpMode() {
        waitForStart();

        motors = findMotors(hardwareMap);

        Motor leftFrontDrive = motors.get(0);
        Motor rightFrontDrive = motors.get(1);
        Motor leftBackDrive = motors.get(2);
        Motor rightBackDrive = motors.get(3);

        c++;
        if (c > 300){
            leftFrontDrive.set(0);
            rightFrontDrive.set(0);
            leftBackDrive.set(0);
            rightBackDrive.set(0);
        }else{
            leftFrontDrive.set(0.5);
            rightFrontDrive.set(0.5);
            leftBackDrive.set(0.5);
            rightBackDrive.set(0.5);
        }
        c++;

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
}
