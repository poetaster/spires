/**
   handle encoder button long press event
*/
void onEb1LongPress(EncoderButton& eb) {
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

  
  if (debug) {
    Serial.print("button1 longPressCount: ");
    Serial.println(eb.longPressCount());
  }
}
/**
   handle encoder turn with  button pressed
   offsets OCR2A
*/
void onEb1PressTurn(EncoderButton& eb) {
  enc_delta = eb.increment();
  
  bank = bank + enc_delta; // freom main class total 
  if ( bank > numBank -1 ) {
    bank = 0;
  } else if ( bank < 0 ) {
    bank = numBank-1;
  }
  //Serial.print("eb1 press inc by: ");
  //Serial.println(bank);
    
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

  // set which bank to select formulas from
  int type = eb.clickCount();

  if (type == 1 ) {
    continuous = true;
        cSpeed = 50000;
  }
  if (type == 2) {
    continuous = false;
    cSpeed = 200000;
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

  //displayUpdate();
  enc_delta = eb.increment();
  
  prog = prog + enc_delta; // freom main class total 
  if ( prog > numProg -1 ) {
    prog = 0;
  } else if ( prog < 0 ) {
    prog = numProg-1;
  }
  //Serial.print("eb1 press inc by: ");
  //Serial.println(prog);
    
  for (uint8_t i = 0; i <= prog; i++) {
     analogWrite(led, 255);
     delay(100);
     analogWrite(led, 0);
     delay(140);
  }

  if (debug) {
    Serial.print("eb1 incremented by: ");
    Serial.println(eb.increment());
    Serial.print("eb1 position is: ");
    Serial.println(prog);
  }
}
