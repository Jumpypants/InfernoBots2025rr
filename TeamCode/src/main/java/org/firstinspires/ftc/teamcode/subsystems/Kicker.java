package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.murphy.MurphyTask;

@Config
public class Kicker {
    public static double OUT_POSITION = 0.34;
    public static double IN_POSITION = 0.0;

    public static double KICK_HALF_TIME = 0.5;

    private final Servo KICKER_SERVO;

    public Kicker(HardwareMap hardwareMap) {
        KICKER_SERVO = hardwareMap.get(Servo.class, "kicker");
    }

    public void setKicker(double position) {
        KICKER_SERVO.setPosition(position);
    }

    public static class KickTask extends MurphyTask {
        private final Kicker kicker;

        public KickTask(Kicker kicker) {
            this.kicker = kicker;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            kicker.setKicker(OUT_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            if (ELAPSED_TIME.seconds() > KICK_HALF_TIME) {
                kicker.setKicker(IN_POSITION);
                return false;
            }
            return true;
        }
    }

    public static class KickActionRR implements Action {
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;
        private final Kicker kicker;

        public KickActionRR (Kicker kicker) {
            this.kicker = kicker;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized) {
                initialized = true;
                elapsedTime.reset();
                kicker.setKicker(OUT_POSITION);
            }

            if (elapsedTime.seconds() > KICK_HALF_TIME) {
                kicker.setKicker(IN_POSITION);
                return false;
            }
            return true;
        }
    }
}
