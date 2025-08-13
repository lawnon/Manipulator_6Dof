Vier Achs Roboter:

Der Roboterprogramm wurde mit c++ geschrieben innerhalb der Arduino-IDE (Integratede
development Environment). Der Mikrocontroller arbeitet Zyklisch und ist
Echtzeitf¨ahig. Die Steuerung besteht aus einer Hauptverarbeitungs-Programm und zwei
Klassen f¨ur die Modellierung der Antriebe und Kinematik.

• Roboter.ino: Hauptverarbeitungs-Programm.
• Joint.cpp: Antriebsverarbeitungs-Klasse.
• Kinematic.cpp: Kinematische Modellierungs-Klasse.

Hauptverarbeitungs-Programm und Datenaustausch:

Dies beinhaltet die Logik f¨ur der Auswertung der gesamten Robotersteuerung. Durch
die Serielle Kommunikation zwischen Mikrocontroller und PC k¨onnen wir Daten austauschen,
und dadurch Befehle an den Roboter senden. Die Befehle sind unten aufgelistet.

1 S e r i a l . p r i n t l n ( ”Incomming   Data :   ” + input ) ;
2
3   int j t 1 Index = input . indexOf ( ”JT1=” ) ;
4   int j t 2 Index = input . indexOf ( ”JT2=” ) ;
5   int j t 3 Index = input . indexOf ( ”JT3=” ) ;
6   int j t 4 Index = input . indexOf ( ”JT4=” ) ;
7   int he r e Index = input . indexOf ( ”Here ” ) ;
8   int homeIndex = input . indexOf ( ”Home” ) ;
9   int posIndex = input . indexOf ( ” Po s i t i on ” ) ;
10  int gotoIndex = input . indexOf ( ”Goto ( ” ) ;
