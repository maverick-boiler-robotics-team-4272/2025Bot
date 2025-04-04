package frc.robot.constants.positions;

import edu.wpi.first.math.geometry.*;


import static frc.robot.constants.FieldConstants.*;

import com.pathplanner.lib.path.PathPlannerPath;

public class PositionContainer {
    public final Pose2d CORAL_AB;
    public final Pose2d CORAL_CD;
    public final Pose2d CORAL_EF;
    public final Pose2d CORAL_GH;
    public final Pose2d CORAL_IJ;
    public final Pose2d CORAL_KL;

    public final Pose2d CORAL_STATION_LEFT;
    public final Pose2d CORAL_STATION_RIGHT;
    public final Pose2d LEFT_BARGE;
    public final Pose2d MIDDLE_BARGE;
    public final Pose2d RIGHT_BARGE;

    public final PathPlannerPath RIGHT_BARGE_PATH, MIDDLE_BARGE_PATH, LEFT_BARGE_PATH;

    public final PathPlannerPath CORAL_A;
    public final PathPlannerPath CORAL_B;
    public final PathPlannerPath CORAL_C;
    public final PathPlannerPath CORAL_D;
    public final PathPlannerPath CORAL_E;
    public final PathPlannerPath CORAL_F;
    public final PathPlannerPath CORAL_G;
    public final PathPlannerPath CORAL_H;
    public final PathPlannerPath CORAL_I;
    public final PathPlannerPath CORAL_J;
    public final PathPlannerPath CORAL_K;
    public final PathPlannerPath CORAL_L;

    public final PathPlannerPath SCORE_AB;
    public final PathPlannerPath SCORE_CD;
    public final PathPlannerPath SCORE_EF;
    public final PathPlannerPath SCORE_GH;
    public final PathPlannerPath SCORE_IJ;
    public final PathPlannerPath SCORE_KL;

    public PositionContainer(boolean red) {
        //Variable = new Pose2d(red ? FIELD_LENGTH_METERS - x : x, y, red ? Rotation2d.fromDegrees(red degrees) : Rotation2d.fromDegrees(blue Degrees))
        //Insert Precomputed positions here
		CORAL_GH = new Pose2d(red ? 11.2542 : FIELD_LENGTH_METERS - 11.2542, red ? 3.9799 : FIELD_WIDTH_METERS - 3.9799, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		CORAL_IJ = new Pose2d(red ? 12.1107 : FIELD_LENGTH_METERS - 12.1107, red ? 2.5525 : FIELD_WIDTH_METERS - 2.5525, red ? Rotation2d.fromDegrees(60) : Rotation2d.fromDegrees(-120));
		CORAL_KL = new Pose2d(red ? 13.9589 : FIELD_LENGTH_METERS - 13.9589, red ? 2.5375 : FIELD_WIDTH_METERS - 2.5375, red ? Rotation2d.fromDegrees(120) : Rotation2d.fromDegrees(-60));
		CORAL_AB = new Pose2d(red ? 14.8153 : FIELD_LENGTH_METERS - 14.8153, red ? 3.9649 : FIELD_WIDTH_METERS - 3.9649, red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
		CORAL_CD = new Pose2d(red ? 13.9589 : FIELD_LENGTH_METERS - 13.9589, red ? 5.5426 : FIELD_WIDTH_METERS - 5.5426, red ? Rotation2d.fromDegrees(-120) : Rotation2d.fromDegrees(60));
		CORAL_EF = new Pose2d(red ? 12.1558 : FIELD_LENGTH_METERS - 12.1558, red ? 5.5426 : FIELD_WIDTH_METERS - 5.5426, red ? Rotation2d.fromDegrees(-60) : Rotation2d.fromDegrees(120));
		CORAL_STATION_LEFT = new Pose2d(red ? 16.3219 : FIELD_LENGTH_METERS - 16.3219, red ? 1.0901 : FIELD_WIDTH_METERS - 1.0901, red ? Rotation2d.fromDegrees(126) : Rotation2d.fromDegrees(-55));
		CORAL_STATION_RIGHT = new Pose2d(red ? 16.2799 : FIELD_LENGTH_METERS - 16.2799, red ? 7.0018 : FIELD_WIDTH_METERS - 7.0018, red ? Rotation2d.fromDegrees(-126) : Rotation2d.fromDegrees(55));
		RIGHT_BARGE = new Pose2d(red ? 9.4878 : FIELD_LENGTH_METERS - 9.4878, red ? 3.1445 : FIELD_WIDTH_METERS - 3.1445, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		MIDDLE_BARGE = new Pose2d(red ? 9.5017 : FIELD_LENGTH_METERS - 9.5017, red ? 2.0265 : FIELD_WIDTH_METERS - 2.0265, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		LEFT_BARGE = new Pose2d(red ? 9.5157 : FIELD_LENGTH_METERS - 9.5157, red ? 0.8805 : FIELD_WIDTH_METERS - 0.8805, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));

        //Stop Precomputed positions here

        try {
            RIGHT_BARGE_PATH = isRedSide() ? PathPlannerPath.fromPathFile("Find Right Barge").flipPath() : PathPlannerPath.fromPathFile("Find Right Barge");
            MIDDLE_BARGE_PATH = isRedSide() ? PathPlannerPath.fromPathFile("Find Middle Barge").flipPath() : PathPlannerPath.fromPathFile("Find Middle Barge");
            LEFT_BARGE_PATH = isRedSide() ? PathPlannerPath.fromPathFile("Find Left Barge").flipPath() : PathPlannerPath.fromPathFile("Find Left Barge");

            CORAL_A = isRedSide() ? PathPlannerPath.fromPathFile("Find A").flipPath() : PathPlannerPath.fromPathFile("Find A");
            CORAL_B = isRedSide() ? PathPlannerPath.fromPathFile("Find B").flipPath() : PathPlannerPath.fromPathFile("Find B");
            CORAL_C = isRedSide() ? PathPlannerPath.fromPathFile("Find C").flipPath() : PathPlannerPath.fromPathFile("Find C");
            CORAL_D = isRedSide() ? PathPlannerPath.fromPathFile("Find D").flipPath() : PathPlannerPath.fromPathFile("Find D");
            CORAL_E = isRedSide() ? PathPlannerPath.fromPathFile("Find E").flipPath() : PathPlannerPath.fromPathFile("Find E");
            CORAL_F = isRedSide() ? PathPlannerPath.fromPathFile("Find F").flipPath() : PathPlannerPath.fromPathFile("Find F");
            CORAL_G = isRedSide() ? PathPlannerPath.fromPathFile("Find G").flipPath() : PathPlannerPath.fromPathFile("Find G");
            CORAL_H = isRedSide() ? PathPlannerPath.fromPathFile("Find H").flipPath() : PathPlannerPath.fromPathFile("Find H");
            CORAL_I = isRedSide() ? PathPlannerPath.fromPathFile("Find I").flipPath() : PathPlannerPath.fromPathFile("Find I");
            CORAL_J = isRedSide() ? PathPlannerPath.fromPathFile("Find J").flipPath() : PathPlannerPath.fromPathFile("Find J");
            CORAL_K = isRedSide() ? PathPlannerPath.fromPathFile("Find K").flipPath() : PathPlannerPath.fromPathFile("Find K");
            CORAL_L = isRedSide() ? PathPlannerPath.fromPathFile("Find L").flipPath() : PathPlannerPath.fromPathFile("Find L");
            
            SCORE_AB = isRedSide() ? PathPlannerPath.fromPathFile("Find AB").flipPath() : PathPlannerPath.fromPathFile("Find AB");
            SCORE_CD = isRedSide() ? PathPlannerPath.fromPathFile("Find CD").flipPath() : PathPlannerPath.fromPathFile("Find CD");
            SCORE_EF = isRedSide() ? PathPlannerPath.fromPathFile("Find EF").flipPath() : PathPlannerPath.fromPathFile("Find EF");
            SCORE_GH = isRedSide() ? PathPlannerPath.fromPathFile("Find GH").flipPath() : PathPlannerPath.fromPathFile("Find GH");
            SCORE_IJ = isRedSide() ? PathPlannerPath.fromPathFile("Find IJ").flipPath() : PathPlannerPath.fromPathFile("Find IJ");
            SCORE_KL = isRedSide() ? PathPlannerPath.fromPathFile("Find KL").flipPath() : PathPlannerPath.fromPathFile("Find KL");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
