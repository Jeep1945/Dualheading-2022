//-------------------------------------------------------------------------------------------------

void button_linemove() {
  // byte ABline_Direction[2] = {0,0};   // {heading, cm of movement}
  // Button_left_S, Button_middle_S, Button_right_S
  if ((ABline_Direction[0] == 0) || (Button_middle_S == "0")) {
    ABline_Direction[0] = heading;
    ABline_Direction[1] = 0;
  }

  if ((Button_left_S == "0") && (millis() - Button_delay40 > 1500)) {
    ABline_Direction[1] += 2;
    Button_delay40 = millis();
  }

  if ((Button_right_S == "0") && (millis() - Button_delay42 > 1500)) {
    ABline_Direction[1] -= 2;
    Button_delay42 = millis();
  }

  if ((abs(ABline_Direction[0] - heading) > 150) || (abs(ABline_Direction[0] - heading) < 50))
    ABline_Direction[2] = 1;
  else
    ABline_Direction[2] = -1;
}
