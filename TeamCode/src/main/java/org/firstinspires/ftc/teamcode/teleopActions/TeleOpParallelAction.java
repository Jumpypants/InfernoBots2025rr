package org.firstinspires.ftc.teamcode.teleopActions;

public class TeleOpParallelAction extends TeleOpAction {
    private final TeleOpAction[] actions;

    private final boolean stopOnFirstCompletion;

    public TeleOpParallelAction(boolean stopOnFirstCompletion, TeleOpAction... actions) {
        super();
        this.actions = actions;
        this.stopOnFirstCompletion = stopOnFirstCompletion;
    }

    @Override
    public boolean step() {
        boolean runAgain = false;

        for (TeleOpAction action : actions) {
            if (action.step()) {
                runAgain = true;
                if (stopOnFirstCompletion) {
                    break;
                }
            }
        }

        return runAgain;
    }
}
