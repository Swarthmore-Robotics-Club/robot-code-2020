/* Fake realities for determining next move */
#define FRONT_SENSOR_LOWEST_READING_IF_WALL_PRESENT 600
#define SIDE_SENSOR_LOWEST_READING_IF_WALL_PRESENT 300

/* Later this will be where a lot of the logic is stored - DFS/FloodFill algorithm stuff */
void deTerminator() {
  if (sumFF/(float)NUM_TIMES_TO_READ_SENSORS < FRONT_SENSOR_LOWEST_READING_IF_WALL_PRESENT) {
    setMotorDirection(true, true);
    setMotorDirection(true, false);
    buzzLightyearLeftThruster = (Forward){leftEncoder + TICKS_PER_CELL, leftEncoder + TICKS_PER_CELL/2, 0, MIN_VOLTAGE};
    prettyPrint("LEFT SETUP FOR FORWARD:\n");
    prettyPrint("\tstart ticks: " + (String)leftEncoder + ", desired ticks: " + (String)(leftEncoder + TICKS_PER_CELL) + ", halfwaypt: " + (String)(leftEncoder + TICKS_PER_CELL/2) + "\n");
    buzzLightyearRightThruster = (Forward){rightEncoder + TICKS_PER_CELL, rightEncoder + TICKS_PER_CELL/2, 0, MIN_VOLTAGE};
    prettyPrint("RIGHT SETUP FOR FORWARD:\n");
    prettyPrint("\tstart ticks: " + (String)rightEncoder + ", desired ticks: " + (String)(rightEncoder + TICKS_PER_CELL) + ", halfwaypt: " + (String)(rightEncoder + TICKS_PER_CELL/2) + "\n");
    occident = FORWARD;
  } else if ((sumBL + sumFL)/(float)NUM_TIMES_TO_READ_SENSORS < 2*SIDE_SENSOR_LOWEST_READING_IF_WALL_PRESENT) { /* Turn left */
    occident = TURN;
  } else if ((sumBR + sumBL)/(float)NUM_TIMES_TO_READ_SENSORS < 2*SIDE_SENSOR_LOWEST_READING_IF_WALL_PRESENT) { /* Turn right */
    occident = TURN;
  } else {
    occident = CHILL;
    prettyPrint("We have walls in front, to the left and to the right. Lets chill.\n");  
  }
}
