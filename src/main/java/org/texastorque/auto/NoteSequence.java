package org.texastorque.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * A NoteSequence is a list of pairs of locations to describe all the little
 * sub paths that an auto can take. 
 * 
 * Note indexes work as so. The first three notes that are placed inside the
 * alliance wing
 * are indexed from top down 1, 2, and 3. The notes on the center line are
 * indexed from top
 * down 10, 20, 30, 40, 50. Close notes are indexed < 10, center line notes are
 * indexed >= 10.
 * 
 * Starting points exist around the subwoofer.
 * 
 * Adapative is super top secret, email <jus@justusl.com>
 */
public class NoteSequence {
    
    public static interface Location {
        public String get();
        public default boolean isMidline() { return false; }
        public default boolean isFarSide() { return false; }
    }

    public static enum StartPoint implements Location {
        // Used to handle a null case etc.
        NONE,

        // Starting locations
        AMP,
        CTR,
        SRC,
        FAR,
        DASH;

        public String get() { return toString(); }
    }

    public static enum NotePoint implements Location {
        // Close notes
        N_1(1),
        N_2(2),
        N_3(3),

        // Midline notes
        N_10(10),
        N_20(20),
        N_30(30),
        N_40(40),
        N_50(50);

        private final int id;
        private NotePoint(int id) { this.id = id; }
        public String get() { return "" + id; }
        public boolean isMidline() { return id >= 10; }
        public boolean isFarSide() { return id >= 20; } 

        @Override
        public String toString() {
            return "" + id;
        }
    }

    // Amp side shooting position = (3.00, 5.55)
    // Far side shooting position = (2.10, 3.33)

    /** TOP SECRET */
    public static enum Adapative implements Location {
        N_10_OR_20(10, 20),
        N_20_OR_10(20, 10),

        N_50_OR_40(50, 40),
        N_40_OR_50(50, 40);

        private final int def, alt;
        private Adapative(int def, int alt) {
            this.def = def;
            this.alt = alt;
        }
        public String get() {
            if (isAlternateRequested())
                return "" + alt;
            return "" + def;
        }
        public boolean isMidline() { return true; }
        public boolean isFarSide() { return def == 40 || def == 50 || alt == 40 || alt == 50; }

        @Override
        public String toString() {
            return "" + def + "/" + alt;
        }
    }

    private static final NetworkTableEntry altEntry = NetworkTableInstance.getDefault().getTable("toast").getEntry("do_alt");

    public static boolean isAlternateRequested() {
        return altEntry.getBoolean(false);
    }

    public static record LocationPair(Location start, Location end) {
        public static final LocationPair NONE = new LocationPair(StartPoint.NONE, StartPoint.NONE);

        private String getPathName() {
            return "go_" + start.get() + "_to_" + end.get();
        } 
        /**
         * Calculates the name of and loads the path that will take the robot from the
         * current note we are at to the next note in the sequence.
         * 
         * @return Some PathPlannerPath object that we should follow.
         */
        public PathPlannerPath getPath() {
            return AutoManager.getInstance().getPath(getPathName());
        }
        /** Look at the first path and check the robots starting position. */
        public Pose2d getStartingPose() {
            return getPath().getPreviewStartingHolonomicPose();
        }

        @Override
        public final boolean equals(final Object other) {
            final LocationPair cast = (LocationPair)other;
            return start.equals(cast.start) && end.equals(cast.end);
        }
    }

    private final List<LocationPair> pairs = new ArrayList<>();
    private final Location[] notes;

    /**
     * Creates a note sequence from a variatic list of arguments which provide
     * indexes.
     */
    public NoteSequence(final Location... notes) {
        this.notes = notes;
        for (int i = 0; i < notes.length - 1; i++) {
            pairs.add(new LocationPair(notes[i], notes[i + 1]));
        }
    }

    @Override
    public String toString() {
        String output = notes[0].toString();
        for (int i = 1; i < notes.length; i++) {
            output += "->" + notes[i].toString();
        }
        return output;
    }

   /**
     * Get and remove the next note pair in the sequence.
     * 
     * @return The next note pair.
     */
    public LocationPair popNext() {
        if (hasNext()) {
            return pairs.remove(0);
        } 
        return LocationPair.NONE;
    }

    /**
     * Take a peek at the next note pair in the sequence, but do not remove it.
     * 
     * @return The next note pair.
     */
    public LocationPair peekNext() {
        if (hasNext()) {
            return pairs.get(0);
        } 
        return LocationPair.NONE;
    }

    /**
     * Do we have another pair in our sequence?
     */
    public boolean hasNext() {
        return pairs.size() > 0;
    }
}
