void rollcalc()
{ // not any more - April 30, 2019 - roll to right is positive Now! Still Important
  double rollnordabs, rollnordrel, rolleastabs, rolleastrel;
  double nord = 11119494.9494,  east = 0, nordWinkel, eastWinkel;
  String norddeci, eastdeci, nordGrad , eastGrad;
  rollzuvor = roll;
  if (AntDistance != 0)  baseline = AntDistance;
  roll = asin(relPosD / baseline) * PI180;
  roll *= -100; // left tilt should be negative
  roll =  constrain(roll, -30, 30);

  rollCorrectionDistance = (sin((roll) / PI180) * tractorhight);  // roll deviation
  baselineHorizontal = (cos((roll) / PI180) * baseline);
  if ((rollaktiv == 0) || (abs(roll) > 30)) {
    rollCorrectionDistance = 0;
    baselineHorizontal = baseline;
    if (rollaktiv == 0) baselineHorizontal = 0;
    // rollCorrectionDistance = 0;
    // baselineHorizontal = 0;
  }
    rollnordabs = (sin(heading  / PI180) * rollCorrectionDistance);
    rollnordabs += (sin(heading  / PI180) * (0.5 * baselineHorizontal));
    rolleastabs = (cos(heading  / PI180) * -rollCorrectionDistance);
    rolleastabs -= (cos(heading  / PI180) * (0.5 * baselineHorizontal));
    // 1° auf Erdkugel == 11119494,9494 cm
    // bei Nord nord = 11119494,9494 cm  /1°
    // bei East east = (cos(nord(heading)) * nord
    norddeci = (GGAnord.substring(2)); //Daten von GGAnord
    nordGrad = (GGAnord.substring(0, 2));
    eastdeci = (GGAeast.substring(3)); //Daten von GGAeast
    eastGrad = (GGAeast.substring(0, 3));
    fixnorddeci = norddeci.toDouble();
    nordWinkel = nordGrad.toDouble();
    fixeastdeci = eastdeci.toDouble();
    eastWinkel = eastGrad.toDouble();
    fixnorddeci = fixnorddeci / 60.0;
    fixeastdeci = fixeastdeci / 60.0;
    double nordWinkel1 = nordWinkel + fixnorddeci;
    // bei Nord nord = 11119494,9494 cm  /1°
    rollnordrel = rollnordabs / nord + fixnorddeci;
    // bei East east = (cos(nord())
    east = cos(nordWinkel1 / PI180) * 637100080;  // the radius of earth by north
    east = (2 * east * 3.141592) / 360;    // cm pro grad
    if (GGAWestEast == "W")  rolleastabs *= -1;
    rolleastrel = rolleastabs / east + fixeastdeci;

    rollnord = rollnordrel * 10000000;
    rolleast = rolleastrel * 10000000;

    rollnordrel = rollnordrel * 60.0;
    rolleastrel = rolleastrel * 60.0;
    rollnord1 = (nordWinkel * 100 + rollnordrel);
    rolleast1 = (eastWinkel * 100 + rolleastrel);
  if (debugmode3)  {
    Serial.println("");
    Serial.print("  fixnorddeci   :" + String(fixnorddeci, 7));
    Serial.print("  fixeastdeci   :" + String(fixeastdeci, 7));
    Serial.print("  rollnordrel   :" + String(rollnordrel, 7));
    Serial.println("  rolleastrel   :" + String(rolleastrel, 7));
    /*    Serial.print("  relPosD 2  :" + String(relPosD));
        Serial.print("  relPosDH   :" + String(relPosDH));
        Serial.print("  rollCorrectionDistance  :" + String(rollCorrectionDistance));
        Serial.print("  roll       :" + String(roll));
        Serial.print("  rollzuvor  :" + String(rollzuvor));
        Serial.println("  roll  :" + String(roll));
    */
  }
} // Ende
