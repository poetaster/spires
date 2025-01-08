/**
   handle encoder button long press event
*/
void onEb1LongPress(EncoderButton& eb) {

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

  int dir = enc_offset + eb.increment();
  dir = constrain(dir, -7, 7 );

  enc_offset = dir;
  if (debug) {
    Serial.print("eb1 press inc by: ");
    Serial.println(eb.increment());
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

  if (debug) {
    Serial.print("bank: ");
    Serial.println(eb.clickCount());
  }
  // displayUpdate();
}

/**
    handle left button short release
*/
void onLeftReleased(EncoderButton& left) {

  if (bank == 1)
  {
    if (pb1 > 1) {
      pb1--;
    } else if (pb1 == 1) {
      pb1 = pb1total;
    }
    prog = pb1;
  } 
  else if (bank == 2) {
    if (pb2 > 1) {
      pb2--;
    } else if (pb2 == 1) {
      pb2 = pb2total;
    }
    prog = pb2;
  } 
  else if (bank == 3) {
    if (pb3 > 1) {
      pb3--;
    } else if (pb3 == 1) {
      pb3 = pb3total;
    }
    prog = pb3;
  }

  if (debug) {
    Serial.print("PROGRAM: ");
    Serial.println(prog);
  }
}

/**
    handle right button short release
*/
void onRightReleased(EncoderButton& right) {
  
  if (bank == 1)
  {
    if (pb1 < pb1total) {
      pb1++;
    } else if (pb1 == pb1total) {
      pb1 = 1;
    }
    prog = pb1;
  } 
  else if (bank == 2) {
    if (pb2 < pb2total) {
      pb2++;
    } else if (pb2 == pb2total) {
      pb2 = 1;
    }
    prog = pb2;
  } 
  else if (bank == 3) {
    if (pb3 < pb2total) {
      pb3++;
    } else if (pb3 == pb3total) {
      pb3 = 1;
    }
    prog = pb3;
  }
  if (debug) {
    Serial.print("PROGRAM: ");
    Serial.println(prog);
  }
}

/**
   A function to handle the 'encoder' event without button
*/
void onEb1Encoder(EncoderButton& eb) {

  //displayUpdate();
  encoder_delta = eb.increment();
  enc_delta = eb.increment();

  prog = constrain(eb.position(), 1, numProg ); // freom main class total progs
  
  //enc_offset = dir;

  if (debug) {
    Serial.print("eb1 incremented by: ");
    Serial.println(eb.increment());
    Serial.print("eb1 position is: ");
    Serial.println(prog);
  }
}
