// ===========================================================
// Datei Utilities.cpp
// Beschreibung: Allgemeine Funktions definitionen
// Autor: Chukwunonso Bob-Anyeji
// Datum: 19.0812025@15-31
// ===========================================================

#include "Utilities.hpp"

namespace Utils
{
    word SetBit16(word var, int8 index)
    {
        return ((1 << index) | var);
    }

    word GetBit16(word var, int8 index)
    {
        return ((1 << index) & var);
    }

    word ResetBit16(word var, int8 index)
    {
        if(GetBit16(var, index) >= OK)
        {
            return ((1 << index) ^ var);
        }
        else
        {
            return var;
        }
    }
}
