# Pumpen_Ventileinheit

zum Flashen des Sketches auf einen Arduino muss dieser via USB-Kabel an einen PC angeschlossen werden.
Wir verwenden zum Programmieren und auch zum Flashen die Arduino IDE (https://www.arduino.cc/en/software)

Für das Hydroponik-Projekt wurde ein Arduino Uno WiFi Rev2 beschafft. Folgende Schritte müssen in der Arduino IDE einmalig durchgeführt werden:
1. ```Werkzeuge > "Board: xyz" > Boardverwalter >``` **arduino megaAVR Boards** installieren
2. ```Werkzeuge > Bibliotheken verwalten >``` **ArduinoMqttClient** und **WiFiNINA** installieren
3. ```Werkzeuge > "Board: xyz" > Arduino megaAVR Boards >``` **Arduino Uno WiFi Rev2** auswählen
4. ggf. muss unter ```Werkzeuge > Ports``` noch der korrekte COM-Port ausgewählt werden, normalerweise passiert dies automatisch, wenn der Arduino angeschlossen ist.
