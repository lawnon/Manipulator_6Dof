/*
 * Program: Logger.cpp
 * Description: contains procedures for fromating and writing
 *               State Date to Serial port
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "Logger.hpp"

namespace Logger{

  void logInit(long  baudRate){
    Serial.begin(baudRate);
    while (!Serial){}
    delay(500);
    Serial.print("C++ Version: ");
    Serial.print(__cplusplus);
    Serial.print("\n");
  }

  int Incoming()
  {
    return Serial.available();
  }

  void encode(String data){
    Serial.println(data); // Daten in Lokalen Serial Monitor ausgeben
    Serial2.write(10); // Neuer Zeilen Anfang Kodieren
    for (uint16 i = 0; i < data.length(); i++) {
      Serial2.write(data.charAt(i));
    }
  }

  void log(const char* text){
    Serial.print("==>");
    Serial.print(text);
    Serial.print("\n");
  }

  void logInt(int digit = -1){
    Serial.print("==>");
    Serial.print(digit);
    Serial.print("\n");
  }

  void logft(float dec = -1){
    Serial.print("==>");
    Serial.print(dec);
    Serial.print("\n");
  }

  //void log(String text = "", String titel = ""){
  void log(const char* text, const char* titel){
    Serial.print("==>");
    Serial.print(titel);
    Serial.print(": ");
    Serial.print(text);
    Serial.print("\n");
    //Serial.println("==>" + titel + ": " + text);
  }

  void log(int digit, const char* titel){
    Serial.print("==>");
    Serial.print(titel);
    Serial.print(": ");
    Serial.print(digit);
    Serial.print("\n");
  }

  void logft(float dec, const char* titel){
    Serial.print("==>");
    Serial.print(dec);
    Serial.print("\n");
  }

  void log(Posture* pt){
    Serial.print("==========================================>\n");
    Serial.print("==>Angular Position of Each Joint\n");
    Serial.print("==>JT1: "); Serial.println(pt->Jt1);
    Serial.print("==>JT2: "); Serial.println(pt->Jt2);
    Serial.print("==>JT3: "); Serial.println(pt->Jt3);
    Serial.print("==>JT4: "); Serial.println(pt->Jt4);
    Serial.print("==>JT5: "); Serial.println(pt->Jt5);
    Serial.print("==>JT6: "); Serial.println(pt->Jt6);
  }

  void logln(Posture* pt){
    Serial.print("==>");
    Serial.print(pt->Jt1); Serial.print(", ");
    Serial.print(pt->Jt2); Serial.print(", ");
    Serial.print(pt->Jt3); Serial.print(", ");
    Serial.print(pt->Jt4); Serial.print(", ");
    Serial.print(pt->Jt5); Serial.print(", ");
    Serial.print(pt->Jt6); Serial.print("\n");
  }

  void logln(Posture* pt, const char* title){
    Serial.print("==>");
    Serial.print(title);  Serial.print(":\n");
    Serial.print(pt->Jt1); Serial.print(", ");
    Serial.print(pt->Jt2); Serial.print(", ");
    Serial.print(pt->Jt3); Serial.print(", ");
    Serial.print(pt->Jt4); Serial.print(", ");
    Serial.print(pt->Jt5); Serial.print(", ");
    Serial.print(pt->Jt6); Serial.print("\n");
  }

  String ptEncode(float val){
    String jt = "";

    String jts = String(round(abs(val)));

    if(jts.length() <= 1){
      jts = "00" + jts;
    }
    if(jts.length() <= 2){
      jts = "0" + jts;
    }

    if(val > 0){
      jt = "+" + jts;
    }
    else {
      jt = "-" + jts;
    }
    return jt;
  }

  void logToSerial(Posture pt){
    String spt1 = ptEncode(pt.Jt1);
    String spt2 = ptEncode(pt.Jt2);
    String spt3 = ptEncode(pt.Jt3);
    String spt4 = ptEncode(pt.Jt4);
    String spt5 = ptEncode(pt.Jt5);
    String spt6 = ptEncode(pt.Jt6);

    encode(spt1 + spt2 + spt3 + spt4 + spt5 + spt6);
  }

  void logToSerial(String input){
    encode(input);
  }

  void log(Position* ps){
    Serial.print("==========================================>\n");
    Serial.print("==>Location and Orientation of Endeffector\n");
    Serial.print("==>X: "); Serial.println(ps->X);
    Serial.print("==>Y: "); Serial.println(ps->Y);
    Serial.print("==>Z: "); Serial.println(ps->Z);
    Serial.print("==>A: "); Serial.println(ps->A);
    Serial.print("==>B: "); Serial.println(ps->B);
    Serial.print("==>C: "); Serial.println(ps->C);
  }

  void logln(Position* ps){
    Serial.print("==>");
    Serial.print(ps->X); Serial.print(", ");
    Serial.print(ps->Y); Serial.print(", ");
    Serial.print(ps->Z); Serial.print(", ");
    Serial.print(ps->A); Serial.print(", ");
    Serial.print(ps->B); Serial.print(", ");
    Serial.print(ps->C); Serial.print("\n");
  }

  void log(Position* ps, const char* title){
    Serial.print("==========================================>\n");
    Serial.print("==>")   ; Serial.println(title);
    Serial.print("==>X: "); Serial.println(ps->X);
    Serial.print("==>Y: "); Serial.println(ps->Y);
    Serial.print("==>Z: "); Serial.println(ps->Z);
    Serial.print("==>A: "); Serial.println(ps->A);
    Serial.print("==>B: "); Serial.println(ps->B);
    Serial.print("==>C: "); Serial.println(ps->C);
  }

  void logln(Position* ps, const char* title){
    Serial.print("==>");
    Serial.print(title); Serial.print(":\n");
    Serial.print(ps->X); Serial.print(", ");
    Serial.print(ps->Y); Serial.print(", ");
    Serial.print(ps->Z); Serial.print(", ");
    Serial.print(ps->A); Serial.print(", ");
    Serial.print(ps->B); Serial.print(", ");
    Serial.print(ps->C); Serial.print("\n");
  }

  void log(Command* cmd){
    Serial.print("==========================================>\n");
    Serial.print("==>ID: ");      Serial.print(cmd->Id);
    Serial.print(" | Name: ");    Serial.print(cmd->Name);
    Serial.print(" | Value: ");   Serial.print(cmd->Value);
    Serial.print(" | Content: "); Serial.print(cmd->Content);
    Serial.print("\n");
  }

  void log(PosData* pdata, PosState state){
    switch (state) {
      case PosState::Position:
        Serial.println("==>" + String(pdata->Identifier) +
                       ": "  + String(pdata->Position.X) +
                       ", "  + String(pdata->Position.Y) +
                       ", "  + String(pdata->Position.Z) +
                       ", "  + String(pdata->Position.A) +
                       ", "  + String(pdata->Position.B) +
                       ", "  + String(pdata->Position.C));
        break;
      case PosState::Posture:
        Serial.println("==>*" + String(pdata->Identifier) +
                       ": "   + String(pdata->Posture.Jt1) +
                       ", "   + String(pdata->Posture.Jt2) +
                       ", "   + String(pdata->Posture.Jt3) +
                       ", "   + String(pdata->Posture.Jt4) +
                       ", "   + String(pdata->Posture.Jt5) +
                       ", "   + String(pdata->Posture.Jt6));
        break;
      case PosState::PosAndPost:
        Serial.println("==>" + String(pdata->Identifier) +
                       ": "  + String(pdata->Position.X) +
                       ", "  + String(pdata->Position.Y) +
                       ", "  + String(pdata->Position.Z) +
                       ", "  + String(pdata->Position.A) +
                       ", "  + String(pdata->Position.B) +
                       ", "  + String(pdata->Position.C) +
                       " | *" + String(pdata->Identifier) +
                       ": "   + String(pdata->Posture.Jt1) +
                       ", "   + String(pdata->Posture.Jt2) +
                       ", "   + String(pdata->Posture.Jt3) +
                       ", "   + String(pdata->Posture.Jt4) +
                       ", "   + String(pdata->Posture.Jt5) +
                       ", "   + String(pdata->Posture.Jt6));
        break;
    }
  }
}
