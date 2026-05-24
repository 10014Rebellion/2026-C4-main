package frc.lib.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RebelButtonBoardRebuilt {
    private static GenericHID bbHalveOne;
    private static GenericHID bbHalveTwo;

    public RebelButtonBoardRebuilt(int firstButtonBoardPort, int secondButtonBoardPort) {
        bbHalveOne = new GenericHID(firstButtonBoardPort);
        bbHalveTwo = new GenericHID(secondButtonBoardPort);
    }

    // TOP SIDE
    public Trigger orangePilotTop() {
        return new Trigger(() -> bbHalveTwo.getRawButton(1));
    }

    public Trigger purplePilotTop() {
        return new Trigger(() -> bbHalveTwo.getRawButton(2));
    }

    public Trigger bluePilotTop() {
        return new Trigger(() -> bbHalveTwo.getRawButton(3));
    }

    // CENTER SIDE
    public Trigger redSquareCenter() {
        return new Trigger(() -> bbHalveOne.getRawButton(5));
    }

    public Trigger greenSquareCenter() {
        return new Trigger(() -> bbHalveTwo.getRawButton(8));
    }

    public Trigger yellowSquareCenter() {
        return new Trigger(() -> bbHalveOne.getRawButton(8));
    }

    public Trigger blueSquareCenter() {
        return new Trigger(() -> bbHalveOne.getRawButton(6));
    }

    // LEFT SIDE
    public Trigger whiteUpwardTriangleLeft() {
        return new Trigger(() -> bbHalveOne.getRawButton(1));
    }

    public Trigger whiteDownwardTriangleLeft() {
        return new Trigger(() -> bbHalveOne.getRawButton(2));
    }

    public Trigger greenDiamondLeft() {
        return new Trigger(() -> bbHalveOne.getRawButton(3));
    }

    public Trigger yellowTriangleLeft() {
        return new Trigger(() -> bbHalveTwo.getRawButton(6));
    }

    public Trigger redTriangleLeft() {
        return new Trigger(() -> bbHalveOne.getRawButton(4));
    } 

    // RIGHT SIDE
    public Trigger whiteSquareRight() {
        return new Trigger(() -> bbHalveTwo.getRawButton(9));
    }

    public Trigger yellowRectangleRight() {
        return new Trigger(() -> bbHalveTwo.getRawButton(7));
    }

    public Trigger blueSquareRight() {
        return new Trigger(() -> bbHalveTwo.getRawButton(10));
    }

    public Trigger yellowTriangleRight() {
        return new Trigger(() -> bbHalveOne.getRawButton(10));
    }

    public Trigger redTriangleRight() {
        return new Trigger(() -> bbHalveOne.getRawButton(9));
    }

    // BOTTOM SIDE
    public Trigger redCircleBottom() {
        return new Trigger(() -> bbHalveTwo.getRawButton(5));
    }

    public Trigger blueCircleBottom() {
        return new Trigger(() -> bbHalveTwo.getRawButton(4));
    }
}
