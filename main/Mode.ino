void GetUp() {
  Serial.println("GetUp!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  GetUpBtState = 1;
}

void ModeOnOff() {
  Serial.println("ModeOnOff!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  Lok = 0;
  Rok = 0;
  Cok = 0;
  if(Mode){
    Mode = 0;
  }else{
    Mode = 1;
  }
}
