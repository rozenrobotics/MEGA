move_stop();
  read();
  readCompassSensor();
  convertedRawValue();

  // if (controlValues.cube_state  == 1) cube_down(), delay(1000), cube_up();
  if (controlValues.winch_state == 0) winch_up();
  if (controlValues.mode == 0) manual();  // manual
  if (controlValues.mode == 1) autonomous();

  Serial.println(controlValues.winch_state);
  move();