/**
   handle encoder button long press event
*/
void onEb1LongPress(EncoderButton& eb) {
  if (sine == true) {
      AD[0].setWave(AD9833_TRIANGLE);
      sine = false;
  } else {
      AD[0].setWave(AD9833_SINE);
      sine = true;
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
  //enc_delta = eb.increment();
  
  //dir = constrain(dir, 0, 7 );
  //encoder_delta = eb.increment();
  //enc_delta = eb.increment();
  //cSpeed = cSpeed + (eb.increment() * 10 );
  //sensors[0].setMeasurementTimingBudget(cSpeed);
  

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
  bank = eb.clickCount();

  if (bank == 1 ) {
    continuous = false;
  }
  if (bank == 2) {
    continuous = true;
  }
  /*
  if ( prog > numProg-1 ) {
    prog = 0;
  } else if ( prog < 0 ) {
    prog = numProg-1;
  }
  for (uint8_t i = 0; i < prog; i++) {
     analogWrite(led, 255);
     delay(150);
     analogWrite(led, 0);
     delay(250);
  }
  */
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
  if ( prog > numProg-1 ) {
    prog = 0;
  } else if ( prog < 0 ) {
    prog = numProg-1;
  }
  //Serial.print("eb1 press inc by: ");
  Serial.println(enc_delta);
    
  for (uint8_t i = 0; i < prog; i++) {
     analogWrite(led, 255);
     delay(100);
     analogWrite(led, 0);
     delay(200);
  }

  if (debug) {
    Serial.print("eb1 incremented by: ");
    Serial.println(eb.increment());
    Serial.print("eb1 position is: ");
    Serial.println(prog);
  }
}
