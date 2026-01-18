
//////////////////////////////////////////////////////////////////////////
// Datei: PositionFactory.hpp
// Beschreibung: Dieser Klasse Beinhaltet Logik zur Erstellung, Vernichtung
//               und Verwalung von Positionen und Stellungen.
//
// Autor: Chukwunonso Bob-Anyeji
// Datum: 11.07.2024@12-00
// Aktualisiert: 27.08.2025@11-46
//////////////////////////////////////////////////////////////////////////

#ifndef POSITIONFACTORY_H_
#define POSITIONFACTORY_H_

#define POS_LIMIT 10

#include "Types.hpp"
#include "Logger.hpp"

class RobFrame;
class PositionFactory
{
  private:
    /// Zeiger Zur RobFrame
    RobFrame* _robFrame;
    /// @brief Postion Array Bereichs abfrage
    /// @return 1 wenn nicht voll -1 wenn voll
    int8_t RangeCheck();
    /// @breif float to Short Int
    /// @param ps Position
    /// @return Short Positionen
    sPosition TosPosition(Position ps);
    /// @brief float to Short Int
    /// @param ps Winkelstellung
    /// @return Short Winkelstellungen
    sPosture TosPosture(Posture ps);
    /// @brief Positions zustand Validieren und Ermitteln
    /// @param content String Position
    /// @param delimiter variable Trennzeichen
    /// @return Positionszusstand
    PosState Decode(String content, char delimiter = ',');
    /// @brief Positionsliste Aktualisieren
    /// @param index Datensatzes
    /// @param params String Datensatz
    /// @state param Positions Type
    /// @return void
    void Update(int index,String& params, PosState state);
    /// @brief Array von Positions Daten
    PosData* _data;
    /// @brief Anzahl definierte Datensatze innerhalb des Arrays
    int _definedCount = 0;
  public:
    /// @brief Klassen-Objekt erstellen
    PositionFactory();
    /// @brief Datenarray Vernichten
    ~PositionFactory();
    /// Initialisierung Speicher Allokation
    /// @param robFrame Zeiger
    /// @return void
    void Init(RobFrame* robFrame);
    /// Initialisierung
    void Setup();
    /// @brief String to Position
    /// @param input string
    /// @param filter Trennzeichen
    /// @return Winkelstellung
    Posture DecodePosture(String input, String filter = ",");
    /// @brief String to Position
    /// @param input string
    /// @param filter Trennzeichen
    /// @return Position
    Position DecodePosition(String input, String filter = ",");
    /// @brief Position, Winkelstellung hinzufügen
    /// @param content String
    /// @param state Type
    /// @param filter Trennzeichen
    /// @return index des hinzugefügte Position
    int& AddPosition(String content, PosState state, char filter, int result);
    /// @brief Datensatzt anzahl
    int& Count();
    void Get(String identifier);
    void Del(String identifier);
    /// @brief Debug info ausgeben
    void LogInfo();
    /// @brief Datensaetzte ausgeben
    /// @param Type
    void LogPositions(PosState state);
    /// @brief Position gemaeße index wiedergeben
    /// @return Position
    Position operator[](int index);
};

#endif // POSITIONFACTORY_H_
