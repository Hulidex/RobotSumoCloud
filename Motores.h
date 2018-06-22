#ifndef __Motores__H__
#define __Motores__H__

//PINES
#define IZQ1 37
#define IZQ2 35
#define DER1 41
#define DER2 39


#define LENTO 60
#define MEDIO 100
#define RAPIDO 255


#include "Arduino.h"

enum tipoMovimiento{
   delante, detras, izquierda, derecha, ini
};



void derecha_adelante();//Entradas 01 hacia delante

void izquierda_adelante();//Entradas 10 hacia atras

void derecha_atras();
void izquierda_atras();
void parar();
void girarDerecha();
void girarIzquierda();
void avanzar();
void retroceder();


void cambiarSentido(tipoMovimiento actual);

#endif
