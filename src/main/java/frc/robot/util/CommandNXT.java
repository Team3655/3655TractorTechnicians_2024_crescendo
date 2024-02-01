package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandNXT extends CommandJoystick {

  public static final int FIRE_STAGE1 = 1;
  public static final int FIRE_STAGE2 = 2;
  public static final int A2 = 3;
  public static final int B1 = 4;
  public static final int D1 = 5;
  public static final int A3_UP = 6;
  public static final int A3_RIGHT = 7;
  public static final int A3_DOWN = 8;
  public static final int A3_LEFT = 9;
  public static final int A3_IN = 10;
  public static final int A4_UP = 11;
  public static final int A4_RIGHT = 12;
  public static final int A4_DOWN = 13;
  public static final int A4_LEFT = 14;
  public static final int A4_IN = 15;
  public static final int C1_UP = 16;
  public static final int C1_RIGHT = 17;
  public static final int C1_DOWN = 18;
  public static final int C1_LEFT = 19;
  public static final int C1_IN = 20;
  public static final int FIRE_PADDLE_UP = 21;
  public static final int FIRE_PADDLE_DOWN = 22;
  public static final int EN1_UP = 23;
  public static final int EN1_DOWN = 24;
  public static final int SW1_UP = 25;
  public static final int SW1_DOWN = 26;
  public static final int SW2_ID = 2;
  public static final int Stick_X = 0;
  public static final int Stick_Y = 1;
  public static final int A1_X = 3;
  public static final int A1_Y = 4;
  public static final int Stick_Z = 5;

  public CommandNXT(int port) {
    super(port);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the first stage is depressed but not when the second is
   */
  public Trigger fireStage1() {
    return this.button(FIRE_STAGE1);
  }

  /**
   * Constructs an event instance around a button's digital signal.
   *
   * @return a new Trigger that is true when the second stage is depressed
   */
  public Trigger fireStage2() {
    return this.button(FIRE_STAGE2);
  }

  /** second paddle pushed up */
  public Trigger firePaddleUp() {
    return this.button(FIRE_PADDLE_UP);
  }

  // second paddle pushed down
  public Trigger firePaddleDown() {
    return this.button(FIRE_PADDLE_DOWN);
  }

  // red button
  public Trigger A2() {
    return this.button(A2);
  }

  // button on top and back of controller
  public Trigger B1() {
    return this.button(B1);
  }

  // button on the bottom and back of controller
  public Trigger D1() {
    return this.button(D1);
  }

  // middle joystick up
  public Trigger A3_UP() {
    return this.button(A3_UP);
  }

  // middle joystick right
  public Trigger A3_RIGHT() {
    return this.button(A3_RIGHT);
  }

  // middle joystick down
  public Trigger A3_DOWN() {
    return this.button(A3_DOWN);
  }

  // middle joystick left
  public Trigger A3_LEFT() {
    return this.button(A3_LEFT);
  }

  // middle joystick in
  public Trigger A3_IN() {
    return this.button(A3_IN);
  }

  // top right joystick up
  public Trigger A4_UP() {
    return this.button(A4_UP);
  }

  // top right joystick in
  public Trigger A4_RIGHT() {
    return this.button(A4_RIGHT);
  }

  // top right joystick down
  public Trigger A4_DOWN() {
    return this.button(A4_DOWN);
  }

  // top rigjt joystick left
  public Trigger A4_LEFT() {
    return this.button(A4_LEFT);
  }

  // top right joystick in
  public Trigger A4_IN() {
    return this.button(A4_IN);
  }

  // left gray stick up
  public Trigger C1_UP() {
    return this.button(C1_UP);
  }

  // left gray stick right
  public Trigger C1_RIGHT() {
    return this.button(C1_RIGHT);
  }

  // left gray stick down
  public Trigger C1_DOWN() {
    return this.button(C1_DOWN);
  }

  // left gray stick left
  public Trigger C1_LEFT() {
    return this.button(C1_LEFT);
  }

  // left gray stick in
  public Trigger C1_IN() {
    return this.button(C1_IN);
  }

  // bottom right wheel up
  public Trigger EN1_UP() {
    return this.button(EN1_UP);
  }

  // bottom right wheel down
  public Trigger EN1_DOWN() {
    return this.button(EN1_DOWN);
  }

  // bottom left wheel up
  public Trigger SW1_UP() {
    return this.button(SW1_UP);
  }

  // bottom left wheel down
  public Trigger SW1_DOWN() {
    return this.button(SW1_DOWN);
  }

  // bottom middle wheel
  public double SW2() {
    return this.getRawAxis(SW2_ID);
  }

  // main stick forward and backward
  public double StickYAxis() {
    return this.getRawAxis(Stick_Y);
  }

  // main stick left and right
  public double StickXAxis() {
    return this.getRawAxis(Stick_X);
  }

  // main stick rotation
  public double StickZAxis() {
    return this.getRawAxis(Stick_Z);
  }

  // top left stick left and right
  public double A1XAxis() {
    return this.getRawAxis(A1_X);
  }

  // top left stick up and down
  public double A1YAxis() {
    return this.getRawAxis(A1_Y);
  }
}
