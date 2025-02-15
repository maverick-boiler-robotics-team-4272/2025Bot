package frc.robot.utils.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoard {
    private CommandGenericHID playerOne;
    private CommandGenericHID playerTwo;

    /**
     * This is used for the mini-pac configured for this game that uses two ports for all of the buttons
     * 
     * @param playerOneId the port of the first controller
     * @param playerTwoId the port of the second controller
     */
    public ButtonBoard(int playerOneId, int playerTwoId) {
        playerOne = new CommandGenericHID(playerOneId);
        playerTwo = new CommandGenericHID(playerTwoId);
    }

    /**
     * Gives the trigger for the button at the button index
     * 
     * @param button the button index of the desired button
     * 
     * @return The command trigger to that button
     */
    public Trigger getButton(int button) {
        if(button > 16) {
            return playerTwo.button(button - 16);
        }

        return playerOne.button(button);
    }

    public CommandGenericHID getPlayerOnePort() {
        return playerOne;
    }

    public CommandGenericHID getPlayerTwoPort() {
        return playerTwo;
    }
}
