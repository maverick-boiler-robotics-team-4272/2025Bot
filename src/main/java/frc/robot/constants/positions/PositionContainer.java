package frc.robot.constants.positions;

import edu.wpi.first.math.geometry.*;


import static frc.robot.constants.FieldConstants.*;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathPlannerPath;

public class PositionContainer {
    public final Pose2d CORAL_AB;
    public final Pose2d CORAL_CD;
    public final Pose2d CORAL_EF;
    public final Pose2d CORAL_GH;
    public final Pose2d CORAL_IJ;
    public final Pose2d CORAL_KL;

    public final Pose2d ALGAE_AB;
    public final Pose2d ALGAE_CD;
    public final Pose2d ALGAE_EF;
    public final Pose2d ALGAE_GH;
    public final Pose2d ALGAE_IJ;
    public final Pose2d ALGAE_KL;
    
    public final Pose2d LEFT_BARGE;
    public final Pose2d MIDDLE_BARGE;
    public final Pose2d RIGHT_BARGE;

    public final Pose2d CORAL_STATION_LEFT_CLOSE_POINT;
    public final Pose2d CORAL_STATION_LEFT_FAR_POINT;
    public final Pose2d CORAL_STATION_RIGHT_CLOSE_POINT;
    public final Pose2d CORAL_STATION_RIGHT_FAR_POINT;

    public final PathPlannerPath RIGHT_BARGE_PATH;
    public final PathPlannerPath MIDDLE_BARGE_PATH;
    public final PathPlannerPath LEFT_BARGE_PATH;

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

    public final PathPlannerPath CORAL_STATION_LEFT_CLOSE;
    public final PathPlannerPath CORAL_STATION_LEFT_FAR;
    public final PathPlannerPath CORAL_STATION_RIGHT_FAR;
    public final PathPlannerPath CORAL_STATION_RIGHT_CLOSE;


    public ArrayList<Pose2d> ALGAE_POSES = new ArrayList<>(6);

    public PositionContainer(boolean red) {
        //Variable = new Pose2d(red ? FIELD_LENGTH_METERS - x : x, y, red ? Rotation2d.fromDegrees(red degrees) : Rotation2d.fromDegrees(blue Degrees))
        //Insert Precomputed positions here
		CORAL_GH = new Pose2d(red ? 11.2542 : FIELD_LENGTH_METERS - 11.2542, red ? 3.9799 : FIELD_WIDTH_METERS - 3.9799, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		CORAL_IJ = new Pose2d(red ? 12.1107 : FIELD_LENGTH_METERS - 12.1107, red ? 2.5525 : FIELD_WIDTH_METERS - 2.5525, red ? Rotation2d.fromDegrees(60) : Rotation2d.fromDegrees(-120));
		CORAL_KL = new Pose2d(red ? 13.9589 : FIELD_LENGTH_METERS - 13.9589, red ? 2.5375 : FIELD_WIDTH_METERS - 2.5375, red ? Rotation2d.fromDegrees(120) : Rotation2d.fromDegrees(-60));
		CORAL_AB = new Pose2d(red ? 14.8153 : FIELD_LENGTH_METERS - 14.8153, red ? 3.9649 : FIELD_WIDTH_METERS - 3.9649, red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
		CORAL_CD = new Pose2d(red ? 13.9589 : FIELD_LENGTH_METERS - 13.9589, red ? 5.5426 : FIELD_WIDTH_METERS - 5.5426, red ? Rotation2d.fromDegrees(-120) : Rotation2d.fromDegrees(60));
		CORAL_EF = new Pose2d(red ? 12.1558 : FIELD_LENGTH_METERS - 12.1558, red ? 5.5426 : FIELD_WIDTH_METERS - 5.5426, red ? Rotation2d.fromDegrees(-60) : Rotation2d.fromDegrees(120));
		CORAL_STATION_LEFT_FAR_POINT = new Pose2d(red ? 15.9624 : FIELD_LENGTH_METERS - 15.9624, red ? 0.7632 : FIELD_WIDTH_METERS - 0.7632, red ? Rotation2d.fromDegrees(126) : Rotation2d.fromDegrees(-55));
		CORAL_STATION_RIGHT_FAR_POINT = new Pose2d(red ? 15.8951 : FIELD_LENGTH_METERS - 15.8951, red ? 7.2868 : FIELD_WIDTH_METERS - 7.2868, red ? Rotation2d.fromDegrees(-126) : Rotation2d.fromDegrees(55));
		RIGHT_BARGE = new Pose2d(red ? 9.4878 : FIELD_LENGTH_METERS - 9.4878, red ? 3.1445 : FIELD_WIDTH_METERS - 3.1445, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		MIDDLE_BARGE = new Pose2d(red ? 9.5017 : FIELD_LENGTH_METERS - 9.5017, red ? 2.0265 : FIELD_WIDTH_METERS - 2.0265, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		LEFT_BARGE = new Pose2d(red ? 9.5157 : FIELD_LENGTH_METERS - 9.5157, red ? 0.8805 : FIELD_WIDTH_METERS - 0.8805, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		CORAL_STATION_RIGHT_CLOSE_POINT = new Pose2d(red ? 16.8380 : FIELD_LENGTH_METERS - 16.8380, red ? 6.6229 : FIELD_WIDTH_METERS - 6.6229, red ? Rotation2d.fromDegrees(-126) : Rotation2d.fromDegrees(55));
		CORAL_STATION_LEFT_CLOSE_POINT = new Pose2d(red ? 16.8284 : FIELD_LENGTH_METERS - 16.8284, red ? 1.3790 : FIELD_WIDTH_METERS - 1.3790, red ? Rotation2d.fromDegrees(126) : Rotation2d.fromDegrees(-55));
		ALGAE_KL = new Pose2d(red ? 13.4694 : FIELD_LENGTH_METERS - 13.4694, red ? 2.7468 : FIELD_WIDTH_METERS - 2.7468, red ? Rotation2d.fromDegrees(120) : Rotation2d.fromDegrees(-60));
		ALGAE_IJ = new Pose2d(red ? 12.1594 : FIELD_LENGTH_METERS - 12.1594, red ? 3.0564 : FIELD_WIDTH_METERS - 3.0564, red ? Rotation2d.fromDegrees(60) : Rotation2d.fromDegrees(-120));
		ALGAE_GH = new Pose2d(red ? 11.7715 : FIELD_LENGTH_METERS - 11.7715, red ? 4.2829 : FIELD_WIDTH_METERS - 4.2829, red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180));
		ALGAE_EF = new Pose2d(red ? 12.6453 : FIELD_LENGTH_METERS - 12.6453, red ? 5.2963 : FIELD_WIDTH_METERS - 5.2963, red ? Rotation2d.fromDegrees(-60) : Rotation2d.fromDegrees(120));
		ALGAE_CD = new Pose2d(red ? 13.9363 : FIELD_LENGTH_METERS - 13.9363, red ? 5.0377 : FIELD_WIDTH_METERS - 5.0377, red ? Rotation2d.fromDegrees(-120) : Rotation2d.fromDegrees(60));
		ALGAE_AB = new Pose2d(red ? 14.3596 : FIELD_LENGTH_METERS - 14.3596, red ? 3.7963 : FIELD_WIDTH_METERS - 3.7963, red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));

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

            CORAL_STATION_LEFT_CLOSE = isRedSide() ? PathPlannerPath.fromPathFile("Find Left Coral Close").flipPath() : PathPlannerPath.fromPathFile("Find Left Coral Close");
            CORAL_STATION_LEFT_FAR = isRedSide() ? PathPlannerPath.fromPathFile("Find Left Coral Far").flipPath() : PathPlannerPath.fromPathFile("Find Left Coral Far");
            CORAL_STATION_RIGHT_CLOSE = isRedSide() ? PathPlannerPath.fromPathFile("Find Right Coral Close").flipPath() : PathPlannerPath.fromPathFile("Find Right Coral Close");
            CORAL_STATION_RIGHT_FAR = isRedSide() ? PathPlannerPath.fromPathFile("Find Right Coral Far").flipPath() : PathPlannerPath.fromPathFile("Find Right Coral Far");

        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        ALGAE_POSES.add(ALGAE_AB);
        ALGAE_POSES.add(ALGAE_CD);
        ALGAE_POSES.add(ALGAE_EF);
        ALGAE_POSES.add(ALGAE_GH);
        ALGAE_POSES.add(ALGAE_IJ);
        ALGAE_POSES.add(ALGAE_KL);
    }
}
