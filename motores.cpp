#include "Motores.h"

tipoMovimiento anterior = ini;


void derecha_adelante(){ //Entradas 01 hacia delante
  digitalWrite(DER1, LOW);
  digitalWrite(DER2, HIGH);
}

void izquierda_adelante(){
  digitalWrite(IZQ1, LOW);
  digitalWrite(IZQ2, HIGH);
}

void derecha_atras(){ //Entradas 10 hacia atras
  digitalWrite(DER1, HIGH);
  digitalWrite(DER2, LOW);
}

void izquierda_atras(){
  digitalWrite(IZQ1, HIGH);
  digitalWrite(IZQ2, LOW);
}

void parar(){
  digitalWrite(IZQ1, LOW);
  digitalWrite(IZQ2, LOW);  
  digitalWrite(DER1, LOW);
  digitalWrite(DER2, LOW);
}

void girarDerecha(){
  cambiarSentido(derecha);
  izquierda_adelante();
  derecha_atras();
}
void girarIzquierda(){
  cambiarSentido(izquierda);
  derecha_adelante();
  izquierda_atras();
}
void avanzar(){
  cambiarSentido(delante);
  derecha_adelante();
  izquierda_adelante();
}
void retroceder(){
  cambiarSentido(detras);
  derecha_atras();
  izquierda_atras();
}


void cambiarSentido(tipoMovimiento actual){
   if (actual != anterior){
      anterior = actual;
      parar();
      delay(100);
    }
}
