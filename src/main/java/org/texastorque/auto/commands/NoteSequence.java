package org.texastorque.auto.commands;

import java.util.ArrayList;
import java.util.List;

import org.texastorque.auto.AutoManager;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * A NoteSequence is a list of notes *indexes* (not actual Note objects) that is
 * encapsulated
 * so that we can calculate paths.
 * 
 * Note indexes work as so. The first three notes that are placed inside the
 * alliance wing
 * are indexed from top down 1, 2, and 3. The notes on the center line are
 * indexed from top
 * down 10, 20, 30, 40, 50. Close notes are indexed < 10, center line notes are
 * indexed >= 10.
 */
public class NoteSequence {
    private final List<Integer> notes = new ArrayList<Integer>();
    private final boolean dash;
    private int lastNote = 0, nextNote = 0;

    /**
     * Creates a note sequence from a variatic list of arguments which provide
     * indexes.
     */
    public NoteSequence(final boolean dash, final int... notes) {
        this.dash = dash;
        for (int note : notes) {
            this.notes.add(note);
        }
    }

    /**
     * Calculates the name of and loads the path that will take the robot from the
     * current
     * note we are at to the next note in the sequence.
     * 
     * @return Some PathPlannerPath object that we should follow.
     */
    public PathPlannerPath getNextPath() {
        lastNote = nextNote;
        nextNote = notes.remove(0);
        final String pathName = "go_" + (dash ? "shoot" : lastNote) + "_to_" + nextNote;
        return AutoManager.getInstance().getPath(pathName);
    }

    public void popNote() {
        lastNote = nextNote;
        nextNote = notes.remove(0);
    }

    /**
     * Take a peek at the index of the next note in the sequence, but do not remove
     * it.
     * 
     * @return The next note's index.
     */
    public int peekNext() {
        return notes.isEmpty() ? 0 : notes.get(0);
    }

    /**
     * Is the next note index a center line note (the index is >= 10)?
     */
    public boolean isNextOnCenterLine() {
        return peekNext() >= 10;
    }

    /**
     * Do we have another note in our sequence?
     */
    public boolean hasNext() {
        return notes.size() > 0;
    }
}
