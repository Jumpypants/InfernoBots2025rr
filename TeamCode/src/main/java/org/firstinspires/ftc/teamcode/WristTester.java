package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeV0;

@TeleOp(name = "--WristTester")

@Config
public class WristTester extends LinearOpMode {
    private Servo WRIST_LEFT_SERVO;
    private Servo WRIST_RIGHT_SERVO;

    private CRServo SPIN_LEFT_SERVO;
    private CRServo SPIN_RIGHT_SERVO;

    private ColorSensor COLOR_SENSOR;
    private DistanceSensor DISTANCE_SENSOR;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        WRIST_LEFT_SERVO = hardwareMap.get(Servo.class, "left");
        WRIST_RIGHT_SERVO = hardwareMap.get(Servo.class, "right");

        SPIN_LEFT_SERVO = new CRServo(hardwareMap, "leftSpin");
        SPIN_LEFT_SERVO.setRunMode(CRServo.RunMode.RawPower);

        SPIN_RIGHT_SERVO = new CRServo(hardwareMap, "rightSpin");
        SPIN_RIGHT_SERVO.setRunMode(CRServo.RunMode.RawPower);

        COLOR_SENSOR = hardwareMap.get(ColorSensor.class, "colorSensor");
        DISTANCE_SENSOR = hardwareMap.get(DistanceSensor.class, "colorSensor");

        ElapsedTime elapsedTime = new ElapsedTime();

        boolean taken = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                setWrist(IntakeV0.WRIST_DOWN_POSITION);
            }

            if (gamepad2.b) {
                setWrist(IntakeV0.WRIST_MID_POSITION);
            }

            if (gamepad2.y) {
                setWrist(IntakeV0.WRIST_UP_POSITION);
            }

            if (gamepad2.right_bumper) {
                setSpin(-0.7);
            }

            if (gamepad2.left_bumper) {
                setSpin(0.7);
            }

            if (DISTANCE_SENSOR.getDistance(DistanceUnit.INCH) < 1) {
                int red = COLOR_SENSOR.red();
                int blue = COLOR_SENSOR.blue();
                int green = COLOR_SENSOR.green();

                if (red > blue && red > green) {
                    elapsedTime.reset();
                    setSpin(0.2);
                    taken = true;
                }
            }

            if (taken && elapsedTime.seconds() > 0.2) {
                taken = false;
                setSpin(0);
            }

            dashboardTelemetry.addData("red", COLOR_SENSOR.red());
            dashboardTelemetry.addData("green", COLOR_SENSOR.green());
            dashboardTelemetry.addData("blue", COLOR_SENSOR.blue());
            dashboardTelemetry.addData("distance", DISTANCE_SENSOR.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.update();

        }
    }

    private void setWrist (double p) {
        WRIST_LEFT_SERVO.setPosition(p);
        WRIST_RIGHT_SERVO.setPosition(1 - p);
    }

    public void setSpin (double p) {
        SPIN_LEFT_SERVO.set(p);
        SPIN_RIGHT_SERVO.set(-p);
    }
}
