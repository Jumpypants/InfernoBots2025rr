package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp(name = "DirectionTest")

public class DirectionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        CRServo frontLeft = new CRServo(hardwareMap, "front-left");
        frontLeft.setInverted(false);

        CRServo frontRight = new CRServo(hardwareMap, "front-right");
        frontRight.setInverted(false);

        Motor backLeft = new Motor(hardwareMap, "back-left");
        backLeft.setInverted(true);

        Motor backRight = new Motor(hardwareMap, "back-right");
        backRight.setInverted(false);

//        Motor armRotation = new Motor(hardwareMap, "elbow-arm");
//        armRotation.setInverted(false);
//
//        Motor armExtension = new Motor(hardwareMap, "erect-arm");
//        armExtension.setInverted(false);
//
//        CRServo intakeLeft = new CRServo(hardwareMap, "intake-left");
//        intakeLeft.setInverted(false);
//
//        CRServo intakeRight = new CRServo(hardwareMap, "intake-right");
//        intakeRight.setInverted(false);

//        CRServo launch = new CRServo(hardwareMap, "airplane-servo");
//        launch.setInverted(false);

        ArrayList<Motor> list = new ArrayList<Motor>();
        list.add(frontLeft);
        list.add(frontRight);
        list.add(backLeft);
        list.add(backRight);
//        list.add(armRotation);
//        list.add(armExtension);
//        list.add(intakeRight);
//        list.add(intakeLeft);
//        list.add(launch);
        for (Motor m : list) {
            m.setRunMode(Motor.RunMode.VelocityControl);
        }

        while (opModeIsActive()) {
            for (Motor m : list) {
                m.set(0.5);
            }
        }
    }
}
