package org.firstinspires.ftc.teamcode.murphy;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class MurphyAction {
    private final ElapsedTime ELAPSED_TIME = new ElapsedTime();

    public MurphyAction() {
        ELAPSED_TIME.reset();
    }

    public abstract boolean step();
}
