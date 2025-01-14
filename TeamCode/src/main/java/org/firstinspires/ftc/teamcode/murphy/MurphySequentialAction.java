package org.firstinspires.ftc.teamcode.murphy;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An action that runs multiple actions in a certain sequence.
 */
public class MurphySequentialAction extends MurphyAction {
    private final MurphyAction[] actions;

    private int currentActionIndex = 0;

    /**
     * @param actions
     * The actions to run in their respective order.
     */
    public MurphySequentialAction(MurphyAction... actions) {
        this.actions = actions;
    }

    @Override
    protected void initialize(Telemetry telemetry) {}

    @Override
    protected boolean run(Telemetry telemetry) {
        if (currentActionIndex >= actions.length) {
            return false;
        }

        if (!actions[currentActionIndex].step(telemetry)) {
            currentActionIndex++;
        }

        return true;
    }
}
