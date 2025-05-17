/**
   handle encoder button long press event
*/
void onEb1LongPress(EncoderButton& eb) {

  long ecount = eb.longPressCount();
  if ( ecount > 1 ) {
    if (sine == true) {
      AD[0].setWave(AD9833_TRIANGLE);
      sine = false;
      analogWrite(led, 255);
      delay(250);
      analogWrite(led, 0);
      delay(250);
    } else {
      AD[0].setWave(AD9833_SINE);
      sine = true;
      analogWrite(led, 255);
      delay(300);
      analogWrite(led, 0);
      delay(300);
    }
  } else {
    allOff();
  }



  if (debug) {
    Serial.print("button1 longPressCount: ");
    Serial.println(sine);
  }
}
/**
   handle encoder turn with  button pressed
   offsets OCR2A
*/
void onEb1PressTurn(EncoderButton& eb) {

  if (debug == false ) allOff();

  enc_delta = eb.increment();

  bank = bank + enc_delta; // freom main class total
  if ( bank > numBank - 1 ) {
    bank = 0;
  } else if ( bank < 0 ) {
    bank = numBank - 1;
  }
  Serial.print("bank: ");
  Serial.println(bank);

  for (uint8_t i = 0; i <= bank; i++) {
    analogWrite(led, 255);
    delay(250);
    analogWrite(led, 0);
    delay(230);
  }

  if (debug) {
    Serial.print("eb1 press inc by: ");
    Serial.println(enc_delta);
    Serial.print("enc_offset is: ");
    Serial.println(enc_offset);
  }
}

/**
   handle encoder turn with  button pressed
*/
void onEb1Clicked(EncoderButton& eb) {

  if ( debug == false )  MIDI.sendNoteOff(frequency_to_midi_note(lastNote), 0, midiChannel);//allOff();

  // set which bank to select formulas from
  int type = eb.clickCount();

  if (bank == 0 ) {

    scaleRoot += 1;
    if (scaleRoot > 6) scaleRoot = 0;

  } else {
    if (type == 1 ) {
      continuous = true;
      cSpeed = 50000;
    }
    if (type == 2) {
      continuous = false;
      cSpeed = 200000;
    }
  }

  for (uint8_t i = 0; i < type; i++) {
    analogWrite(led, 255);
    delay(75);
    analogWrite(led, 0);
    delay(100);
  }
  if (debug) {
    Serial.print("bank: ");
    Serial.println(eb.clickCount());
  }
  // displayUpdate();
}



/**
   A function to handle the 'encoder' event without button
*/
void onEb1Encoder(EncoderButton& eb) {

  // turn off midi notes
  //if (debug == false) allOff();

  //displayUpdate();
  enc_delta = eb.increment();

  //prog = prog + enc_delta; // freom main class total

  if (bank == 0)
  {
    pb1 = pb1 + enc_delta;
    if (pb1 > pb1total - 1) {
      pb1 = 0;
    } else if (pb1 < 0) {
      pb1 = pb1total - 1;
    }
    prog = pb1;
  }
  else if (bank == 1) {
    pb2 = pb2 + enc_delta;
    if (pb2 > pb2total - 1) {
      pb2 = 0;
    } else if (pb2 < 0) {
      pb2 = pb2total - 1;
    }
    prog = pb2;
  }
  else if (bank == 2) {
    pb3 = pb3 + enc_delta;
    if (pb3 > pb3total - 1) {
      pb3 = 0;
    } else if (pb3 < 0) {
      pb3 = pb3total - 1;
    }
    prog = pb3;
  }
  else if (bank == 3) {
    pb4 = pb4 + enc_delta;
    if (pb4 > pb4total - 1) {
      pb4 = 0;
    } else if (pb4 < 0) {
      pb4 = pb4total - 1;
    }
    prog = pb4;
  }
  else if (bank == 4) {
    pb5 = pb5 + enc_delta;
    if (pb5 > pb5total - 1) {
      pb5 = 0;
    } else if (pb5 < 0) {
      pb5 = pb5total - 1;
    }
    prog = pb5;
  }
  else if (bank == 5) {
    pb6 = pb6 + enc_delta;
    if (pb6 > pb6total - 1) {
      pb6 = 0;
    } else if (pb6 < 0) {
      pb6 = pb6total - 1;
    }
    prog = pb6;
  }

  for (uint8_t i = 0; i <= prog; i++) {
    analogWrite(led, 255);
    delay(180);
    analogWrite(led, 0);
    delay(160);
  }

  if (debug) {
    Serial.print("eb1 incremented by: ");
    Serial.println(eb.increment());
    Serial.print("eb1 position is: ");
    Serial.println(prog);
  }
}
