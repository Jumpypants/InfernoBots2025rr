package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

public class Robot {
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public final Intake intake;
    public final Outtake outtake;
    public final MecanumDrive driveBase;
    public final IMU imu;


    public Robot(Gamepad gamepad1, Gamepad gamepad2, Intake intake, Outtake outtake, MecanumDrive driveBase, IMU imu) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;
        this.driveBase = driveBase;
        this.imu = imu;
    }
}
