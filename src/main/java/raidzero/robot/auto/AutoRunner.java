package raidzero.robot.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import raidzero.robot.operations.sequences.*;
import raidzero.robot.dashboard.Tab;

/**
 * Class that manages autonomous sequences.
 */
public class AutoRunner {

    private SendableChooser<AutoSequence> chooser;

    private AutoSequence selectedSequence;

    // list of every sequence 
    private AutoSequence[] availableSequences = {
        // list of all available sequences
    };

    public AutoRunner() {
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("None", new EmptySequence());
        for (AutoSequence sequence : availableSequences) {
            chooser.addOption(sequence.getName(), sequence);
        }
        Shuffleboard.getTab(Tab.MAIN)
                .add("Auton Selection", chooser)
                .withSize(3, 1)
                .withPosition(2, 1);
        // SmartDashboard.putData("Auton Selection", chooser);
    }

    /**
     * Reads the selected autonomous sequence from the SendableChooser.
     */
    public void readSendableSequence() {
        selectSequence(chooser.getSelected());
    }

    /**
     * Selects an autonomous sequence to run.
     * 
     * @param sequence the autonomous sequence to run
     */
    public void selectSequence(AutoSequence sequence) {
        selectedSequence = sequence;
    }

    /**
     * Returns the currently selected autonomous sequence.
     * 
     * @return the selected sequence - null means no sequence.
     */
    public AutoSequence getSelectedSequence() {
        return selectedSequence;
    }

    /**
     * Starts the selected autonomous sequence.
     */
    public void start() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Starting auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.run();
        }
    }

    /**
     * Stops the selected autonomous sequence.
     */
    public void stop() {
        if (selectedSequence != null) {
            System.out.println("[Auto] Stopping auto sequence '" + selectedSequence.getName() + "'...");
            selectedSequence.stop();
        }
    }

    /**
     * Updates the selected autonomous sequence.
     * 
     * @param timestamp
     */
    public void onLoop(double timestamp) {
        if (selectedSequence != null) {
            selectedSequence.onLoop(timestamp);
        }
    }
}