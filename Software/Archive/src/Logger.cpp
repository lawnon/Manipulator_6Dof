/*
 * Program: Logger.cpp
 * Description: contains procedures for fromating and writing
 *               State Date to Serial port
 * Autor: Chukwunonso Bob-Anyeji
 * Date: 09.06.2024
 */

#include "Logger.hpp"

void encode(String data){
  Serial.println(data); // Daten in Lokalen Serial Monitor ausgeben
  Serial2.write(10); // Neuer Zeilen Anfang Kodieren
  for (int i = 0; i < data.length(); i++) {
    Serial2.write(data.charAt(i));
  }
}

void log(String text = ""){
  Serial.println("==>" + text);
}

void logInt(int digit = -1){
  Serial.println("==>" + String(digit));
}
void logft(float dec = -1){
  Serial.println("==>" + String(dec));
}

void log(String text = "", String titel = ""){
  Serial.println("==>" + titel + ": " + text);
}

void log(int digit = -1, String titel = ""){
  Serial.println("==>" + titel + ": " + String(digit));
}
void logft(float dec = -1.0, String titel = ""){
  Serial.println("==>" + titel + ": " + String(dec));
}

void log(Posture pt){
  Serial.println("===================================>");
  Serial.println("==>Angular Position of Each Joint");
  Serial.println("==>JT1: " + String(pt.Jt1));
  Serial.println("==>JT2: " + String(pt.Jt2));
  Serial.println("==>JT3: " + String(pt.Jt3));
  Serial.println("==>JT4: " + String(pt.Jt4));
}

void logln(Posture pt){
  Serial.println("==>" + String(pt.Jt1) +
                 ", " + String(pt.Jt2) +
                 ", " + String(pt.Jt3) +
                 ", " + String(pt.Jt4));
}

void logln(Posture pt, String title){
  Serial.println("==>" + title +
                 ": " + String(pt.Jt1) +
                 ", " + String(pt.Jt2) +
                 ", " + String(pt.Jt3) +
                 ", " + String(pt.Jt4));
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

  encode(spt1 + spt2 + spt3 + spt4);
}

void logToSerial(String input){
  encode(input);
}

void log(Position ps){
  Serial.println("==========================================>");
  Serial.println("==>Location and Orientation of Endeffector");
  Serial.println("==>X: " + String(ps.X));
  Serial.println("==>Y: " + String(ps.Y));
  Serial.println("==>Z: " + String(ps.Z));
  Serial.println("==>A: " + String(ps.A));
  Serial.println("==>B: " + String(ps.B));
  Serial.println("==>C: " + String(ps.C));
}

void logln(Position ps){
  Serial.println("==>" + String(ps.X) +
                 ", " + String(ps.Y) +
                 ", " + String(ps.Z) +
                 ", " + String(ps.A) +
                 ", " + String(ps.B) +
                 ", " + String(ps.C));
}

void log(Position ps, String title="Location and Orientation of Endeffector"){
  Serial.println("==========================================>");
  Serial.println("==>" + title);
  Serial.println("==>X: " + String(ps.X));
  Serial.println("==>Y: " + String(ps.Y));
  Serial.println("==>Z: " + String(ps.Z));
  Serial.println("==>A: " + String(ps.A));
  Serial.println("==>B: " + String(ps.B));
  Serial.println("==>C: " + String(ps.C));
}

void logln(Position ps, String title){
  Serial.println("==>" + title +
                 ": " + String(ps.X) +
                 ", " + String(ps.Y) +
                 ", " + String(ps.Z) +
                 ", " + String(ps.A) +
                 ", " + String(ps.B) +
                 ", " + String(ps.C));
}

void log(Command cmd){
  Serial.println("==================================================>");
  Serial.println("==>ID: " + String(cmd.Id) +
                 "| Name: " + cmd.Name +
                 "| Value: " + String(cmd.Value) +
                 "| Content: " + cmd.Content);
}

void log(PosData pdata, PosState state){
  switch (state) {
  case PosState::Position:
    Serial.println("==>" + String(pdata.Identifier) +
                   ": " + String(pdata.Position.X) +
                   ", " + String(pdata.Position.Y) +
                   ", " + String(pdata.Position.Z) +
                   ", " + String(pdata.Position.A) +
                   ", " + String(pdata.Position.B) +
                   ", " + String(pdata.Position.C));
    break;
  case PosState::Posture:
    Serial.println("==>*" + String(pdata.Identifier) +
                   ": " + String(pdata.Posture.Jt1) +
                   ", " + String(pdata.Posture.Jt2) +
                   ", " + String(pdata.Posture.Jt3) +
                   ", " + String(pdata.Posture.Jt4));
    break;
  case PosState::PosAndPost:
    Serial.println("==>" + String(pdata.Identifier) +
                   ": " + String(pdata.Position.X) +
                   ", " + String(pdata.Position.Y) +
                   ", " + String(pdata.Position.Z) +
                   ", " + String(pdata.Position.A) +
                   ", " + String(pdata.Position.B) +
                   ", " + String(pdata.Position.C) +
                   " | *" + String(pdata.Identifier) +
                   ": " + String(pdata.Posture.Jt1) +
                   ", " + String(pdata.Posture.Jt2) +
                   ", " + String(pdata.Posture.Jt3) +
                   ", " + String(pdata.Posture.Jt4));
   break;
  }
}
