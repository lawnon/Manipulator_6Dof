// ==============================================================
// Datei: Utilities.hpp
// Beschreibung: Header Datei, allgemeine Funktions definitionen
// Autor: Chukwunonso Bob-Anyeji
// Datum: 19.0812025@15-31
// ==============================================================

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "Types.hpp"

namespace Utils
{
    word SetBit16(word var, int8 index);
    word GetBit16(word var, int8 index);
    word ResetBit16(word var, int8 index);
};

#endif // UTILITIES_H_
