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
  
  //prog = prog + enc_delta; // freom main class total 
  
  if (bank == 0)
  {
    pb1 = pb1 + enc_delta;
    if (pb1 == pb1total) {
      pb1 = 1;
    } else if (pb1 < 0) {
      pb1 = pb1total-1;
    }
    prog = pb1;
  } 
  else if (bank == 1) {
    pb2 = pb2 + enc_delta;
    if (pb2 == pb2total) {
      pb2 = 1;
    } else if (pb2 < 0) {
      pb2 = pb2total - 1;
    }
    prog = pb2;
  } 
  else if (bank == 2) {
     pb3 = pb3 + enc_delta;
    if (pb3 == pb2total) {
      pb3 = 1;
    } else if (pb3 < 0) {
      pb3 = pb2total -1;
    }
    prog = pb3;
  }
  Serial.print("prog: ");
  Serial.println(prog);
    
  for (uint8_t i = 0; i <= prog; i++) {
     analogWrite(led, 255);
     delay(80);
     analogWrite(led, 0);
     delay(100);
  }

  if (debug) {
    Serial.print("eb1 incremented by: ");
    Serial.println(eb.increment());
    Serial.print("eb1 position is: ");
    Serial.println(prog);
  }
}
