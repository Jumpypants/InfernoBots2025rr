package org.firstinspires.ftc.teamcode.teleopActions;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TeleOpAction {
    private final ElapsedTime ELAPSED_TIME = new ElapsedTime();

    public TeleOpAction() {
        ELAPSED_TIME.reset();
    }

    public abstract boolean step();
}
