import serial
ser = serial.Serial('/dev/ttyACM0', 9600)

#Funktion: Fuehrt eine Messung durch
#Rueckgabe: Distanz zu dem nächsten Gegenstand def GetDistance():
ser.write('ping()') return ser.readline()

#Funktion: Führt einen 140 Grad Scan durch
#Rueckgabe: Array mit Entfernungsdaten def Fullscan():
return

#Funktion: Soll einen Radarbild ausgeben
#def Radarscan():
#  return

###Gyrosensor###

#Funktion: Temperatur messung
#Rueckgabe: Temperatur in °C def getTemperature():
ser.write('getTemp()') return ser.readline()

#Funktion: Soll prüfen ob Hexy gerade steht
#Rueckgabe: true = Hexy steht grade def isHorizontal():
ser.write('getEuler()') return

#Funktion: Soll die aktuelle Hoehe angeben gemessen NN
#Rueckgabe: Höhe def Level():
ser.write('getAlt()')
return ser.readline()

#Funktion: sendet true falls interrupt
#Rueckgabe: true fuer interrupt def interrupt():
return

#Funktion: Gibt die Blickrichtung des Hexy zurück
#Rueckgabe: Richtung def Heading():
ser.write('getHeading()') return ser.readline()

###Kamera###

#Funktion: Soll Kanten in einer bestimmten Reichweite erkennen
#Rueckgabe: true = in der Entfernung befindet sich eine Kante def isEdge(distance):
return

#Funktion: Soll Prüfen ob die Kamera einsatzbereit ist
#Rueckgabe: true = Kamera ist blockiert ( zu dunkel hell etc.) def camBlocked():
return

#Funktion: Soll Hindernisse erkennen
#Rueckgabe: True = Hindernis befindet sich in der angegebenen Distanz def isObstacle():
return

#Funktion: Gibt die Breite eines Gegenstandes in einer bestimmten entfernung wieder
#Rueckgabe: Breite des Gegestandes 0 wenn kein Gegenstand gefunden -1 wenn
#der Gegenstad den Rand des Bildes Überschreitet
#def wideOfObject(distance):
#	return

#Funktion: gibt die Länge einer Kante wieder
#Rueckgabe: Länge der Kante 0 wenn keine kante gefunden -1 wenn
#die Kante den Rand des Bildes Überschreitet
#def wideOfEdge(distance):
#	return

Programmebene unseres Hauptprogrammes:	
#include "Servotor32.h" Servotor32 hexy;

//Folgende Librarys werden für den 10DOF Sensor benötigt
#include <Wire.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <HMC5883L.h>
#include <BMP085.h>

//Initialisieren für 10DOF Sensor float angles[3]; // yaw pitch roll short heading;
BMP085 dps = BMP085();
long Temp = 0, Pres = 0, Alt = 0;

//Legt den FreeSixIMU fest FreeSixIMU sixDOF = FreeSixIMU();

HMC5883L compass;

//Pineinstellungen
#define sender 17 // Sender Pin 17
#define echo 11	// empfänger Pin 11
#define servopin 31 // Servo Pin

//Servoeinstellungen
#define maxangle 2455
#define minangle 650

//Globaler serialin Speicher char string[12];

//Startfunktion void setup(void) {
//Ultraschallsensor
pinMode(sender, OUTPUT); // legt den pin als sender fest pinMode(echo, INPUT); // legt den pin als empfänger fest

hexy.begin();

//10DOF
Serial.begin(19200); Wire.begin(); delay(1000); dps.init(); dps.dumpCalData(); delay(5000);

delay(5);
sixDOF.init(); //inizialisiert den Accelorator und den Gyro delay(5);
compass = HMC5883L(); //inizialisiert HMC5883(Compass)
}

void loop() {
serialread(); // wartet auf eine Eingabe
//Eingabe wird verglichen mit den verfügbaren Funktionen if(equal("getTemp()"))
{
Serial.println(getTemp());
}
if(equal("getPres()"))
{
Serial.println(getPres());
}
if(equal("getAlt()"))
{
Serial.println(getAlt());
}
if(equal("getHeading()"))
{
Serial.println(getHeading());
}
if(equal("getEuler()"))
{
getEuler();
}
if(equal("ping()"))
{

Serial.println(hexy.ping);
}
}

//Funktion: Vergleicht zwei Arrays
//Rueckgabe: true wenn beide Zweichenketten gleich sind boolean equal(char x[]){
byte i = 0;
while(x[i] != '\0' && string[i] != '\0')
{
if(x[i] != string[i])
{
return false;
}
i++;
}
if(x[i] != '\0' || string[i] != '\0')
{
return false;
}
return true;
}

//Funktion: Liest den String aus dem Serial
//Rueckgabe: Stringeingabe void serialread(){
byte index = 0; while(Serial.available() <= 0)
{
delay(10);
}
while(Serial.available() > 0)
{
string[index] = Serial.read(); index++;
string[index] = '\0';
}
}

//Funktion: Temperaturmessung
//Rueckgabe: Temperatur als Integer long getTemp(){ dps.getTemperature(&Temp);
return Temp;
}

//Funktion: Druckmessung
//Rueckgabe: Druck als Long long getPres(){ dps.getPressure(&Pres); return Pres;
}

//Funktion: Hoehenmessung
//Rueckgabe: Hoehe in cm als Long long getAlt(){ dps.getAltitude(&Alt);
return Alt;
}

//Funktion: Wasserwage
//Keine Rueckgabe, da Ergebniss direkt in den Serial uebergaben wird void getEuler(){
sixDOF.getEuler(angles); Serial.print(angles[0]); Serial.print(" "); Serial.print(angles[1]); Serial.print(" "); Serial.println(angles[2]);
}

//Funktion: Gibt die Richtung an in die Hexy guckt
//Rueckgabe: Compasswert als integer int getHeading(){
// Retrive the raw values from the compass (not scaled). MagnetometerRaw raw = compass.ReadRawAxis();
// Retrived the scaled values from the compass (scaled to the configured scale). MagnetometerScaled scaled = compass.ReadScaledAxis();

// Values are accessed like so:
int MilliGauss_OnThe_XAxis = scaled.XAxis; // (or YAxis, or ZAxis)

// Calculate heading when the magnetometer is level, then correct for signs of axis. heading = atan2(scaled.YAxis, scaled.XAxis);

float declinationAngle = 0.0457; heading += declinationAngle;

// Correct for when signs are reversed. if(heading < 0)
heading += 2*PI;

// Check for wrap due to addition of declination. if(heading > 2*PI)
heading -= 2*PI;

// Convert radians to degrees for readability. return heading * 180/M_PI;
}

//Funktion: Fuehrt eine Distanzmessung durch
//Rueckgabe: Entfernung in cm unsigned short int ping(){ hexy.delay_ms(100); digitalWrite(sender, LOW); hexy.delay_us(2);
digitalWrite(sender, HIGH); //sendet über 10ms ein Signal aus hexy.delay_us(10);
digitalWrite(sender, LOW);
unsigned short int dauer = newpulseIn(echo, HIGH, 500) / 2 ; //Zeit halbieren wegen doppelten "eg den der Schall hat
return (dauer / 34); // umrechnung von Zeit in die zurrueckgelegte Strecke in cm  ( Schallgeschwindigkeit = 340 m/s = 34 cm/ms )
}

//Funktion: wertet das Eingangssignal aus
//Rueckgabe: Dauer von der Messung in ms
//Wurde aus der Library Kopiert, da das Implemetieren der Library Fehler verursacht. unsigned long newpulseIn(uint8_t pin, uint8_t state, unsigned long timeout){
uint8_t bit = digitalPinToBitMask(11);

uint8_t port = digitalPinToPort(11); uint8_t stateMask = (HIGH ? bit : 0);

unsigned long startCount = 0; unsigned long endCount = 0;
unsigned long width = 0; // keep initialization out of time critical area

// convert the 1 from microseconds to a number of times through
// the initial loop; it takes 16 clock cycles per iteration. unsigned long numloops = 0;
unsigned long maxloops = 500;

//Wartet auf das Ende vom vorherigen Impuls
while ((*portInputRegister(port) & bit) == stateMask) if (numloops++ == maxloops)
return 0;

//Wartet auf den Start der Messung
while ((*portInputRegister(port) & bit) != stateMask) if (numloops++ == maxloops)
return 0;

startCount = hexy.micros_new();
// Wartet auf das Eingangssignal und Speichert die Zeit ab while ((*portInputRegister(port) & bit) == stateMask) {
if (numloops++ == maxloops) return 0;
delayMicroseconds(10); //loop 'jams' without this if((hexy.micros_new() - startCount) > 58000 ){ // 58000 = 1000CM return 0;
break;
}
}
return hexy.micros_new() - startCount;
 

