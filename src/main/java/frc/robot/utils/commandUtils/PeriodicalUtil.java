package frc.robot.utils.commandUtils;

import java.util.*;

public class PeriodicalUtil {
    private static List<Periodical> periodicals = new ArrayList<>(1);

    public static void registerPeriodic(Periodical periodic) {
        periodicals.add(periodic);
    }

    //only used for limelights... sooooo
    //otherwise it would be:
    /*
     * for (Periodical p : periodicals) {
     *  p.periodic();
     * }
     */
    public static void runPeriodics() {
        periodicals.get(0).periodic();
    }
}
