package org.firstinspires.ftc.teamcode.murphy;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * An action that runs multiple actions in parallel.
 */
public class MurphyParallelAction extends MurphyAction {
    private final MurphyAction[] actions;

    private final boolean stopOnFirstCompletion;

    /**
     * @param stopOnFirstCompletion
     * Defines weather or not to stop when one of the actions is completed.
     * The alternative is waiting until all actions are completed to stop.
     * @param actions
     * The actions to run.
     */
    public MurphyParallelAction(boolean stopOnFirstCompletion, MurphyAction... actions) {
        this.actions = actions;
        this.stopOnFirstCompletion = stopOnFirstCompletion;
    }

    @Override
    protected void initialize(Telemetry telemetry) {}

    @Override
    protected boolean run(Telemetry telemetry) {
        boolean runAgain = false;

        for (MurphyAction action : actions) {
            if (action.step(telemetry)) {
                runAgain = true;
                if (stopOnFirstCompletion) {
                    break;
                }
            }
        }

        return runAgain;
    }
}
