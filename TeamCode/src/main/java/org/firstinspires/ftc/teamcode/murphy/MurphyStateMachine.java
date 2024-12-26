package org.firstinspires.ftc.teamcode.murphy;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MurphyStateMachine {
    private MurphyState currentState;

    public MurphyStateMachine(MurphyState initialState) {
        currentState = initialState;
    }

    public void step(Telemetry telemetry) {
        telemetry.addData("State", currentState.getName());
        currentState = currentState.step();
    }
}
