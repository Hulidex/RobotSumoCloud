#include "Motores.h"

//RENOMBRADO DE PINES
#define sFRENTE 21
#define sDERECH 19
#define sIZQUIE 20
#define TRIGGER 48
#define PWM 7
#define INFRADELANTE 3
#define INFRADETRAS 2



//VALORES CRÍTICOS DEL SISTEMA
#define RETARDO_INI 3000 //Fija el retardo (en milisegundos) en el cual en robot estará detenido.
#define distancia_localizado 40 //distancia en cm apartir de la cual consideramos que hemos localizado el objetivo
#define distancia_ataque 20 //distancia en cm apartir de la cual embestimos
#define distancia_minima 4 //distancia minimo, a partir de este valor descartamos mediciones
#define PISADO_DELANTE 0
#define PISADO_DETRAS 1
#define PISADAS_NINGUNA 2


enum State{
  Inicial,
  BuscandoEnemigo,
  Avanzando,
  GirandoDerecha,
  GirandoIzquierda,
  HuirLineaDelante,
  HuirLineaDetras,
  HuyendoMarchaDetras,
  HuyendoMarchaDelate,
  HuyendoGirarDerecha
};


//VARIABLES GLOBALES DEL SISTEMA
byte velocidad;
byte contador = 0;
volatile byte duracion;
volatile bool duracionFijada = false;//flag para indicar que la variable duracion ha sido fijada
volatile unsigned long ini_Fre, ini_Der, ini_Izq, end_Fre, end_Der, end_Izq;
volatile bool medido_Der, medido_Fre, medido_Izq;
State estadoActual;







void StartMeasure(){
  medido_Der = false;
  medido_Fre = false;
  medido_Izq = false;
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
}


unsigned int getRange(unsigned long inicio, unsigned long fin) {
  return (fin - inicio) / 58; //comprobar 148
}


byte AlgoPisado(){ 
  auto delante = digitalRead(INFRADELANTE);
  auto detras = digitalRead(INFRADETRAS);
  byte pisado;


  if (delante == LOW && detras == LOW)
    pisado = PISADAS_NINGUNA;
  else if (delante == HIGH)
    pisado = PISADO_DELANTE;
  else
    pisado = PISADO_DETRAS;


  return pisado;
}

bool ComprobarLineas(State &estado){
  bool algoPisado;

  
  switch (AlgoPisado()){
    case  PISADAS_NINGUNA:
      algoPisado = false;
      break;
    case PISADO_DELANTE:
      estado = HuirLineaDelante;
      algoPisado = true;
      break;
    case PISADO_DETRAS:
      estado = HuirLineaDetras;
      algoPisado = true;
      break;
  }


  return algoPisado;
}


void setup() {
  //Empezamos con los motores parados
  parar();
  //Ajustamos timer0 para controlar la velocidad de los motores: se produce la interrupción cada 0.078 ms Formula:
  //Simulamos una frecuenia de 50 Hz aprox: 0.078*255 = 19.92 ms → 0.01902 seg →  50.2 Hz
  /**
   * |CSx2|CSx1|CSx0|     DESCRIPCION       |
   * |0   |0   |0   |  Timer2 deshabilitado |
   * |0   |0   |1   |  Prescaler 1          |
   * |0   |1   |0   |  Prescaler 8          |
   * |0   |1   |1   |  Prescaler 64         |
   * |1   |0   |0   |  Prescaler 256        |
   * |1   |0   |1   |  Prescaler 1024       |
   */
  cli();//stop interrupts
 //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 155;
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
 
  Serial.begin(9600);
  pinMode(sFRENTE, INPUT);
  pinMode(sDERECH, INPUT);
  pinMode(sIZQUIE, INPUT);
  pinMode(TRIGGER, OUTPUT);
  pinMode(INFRADELANTE, INPUT);
  pinMode(INFRADETRAS, INPUT);
  digitalWrite(TRIGGER, LOW);


  
  
  //Interrupciones para medir con los HC-SR04
  attachInterrupt( digitalPinToInterrupt(sFRENTE), ISR_sFRENTE, CHANGE);
  attachInterrupt( digitalPinToInterrupt(sDERECH), ISR_sDERECH, CHANGE);
  attachInterrupt( digitalPinToInterrupt(sIZQUIE), ISR_sIZQUIE, CHANGE);
  sei(); //habilitamos interrupciones.


  estadoActual = Inicial;
}



void loop() {
  //VARIABLES LOCALES
  unsigned int Front;
  unsigned int Der;
  unsigned int Izq;
  byte Velocidad = LENTO;
  bool medicionIniciada = false; //flag que indica si una medicion ha sido solicitada



  while(true){
      switch(estadoActual){
        
        
        
        case Inicial:
          //Serial.println("Inicial");
        
          if (!medicionIniciada){
            StartMeasure();
            medicionIniciada = true;
          }

          if (medido_Fre && medido_Der && medido_Izq){
            medicionIniciada = false;
            
            Front = getRange(ini_Fre, end_Fre);
            Der = getRange(ini_Der, end_Der);
            Izq = getRange(ini_Izq, end_Izq);

            //Serial.print("Medida sFRENTE cm:");
            //Serial.println(getRange(ini_Fre, end_Fre));
            //Serial.print("Medida sDERECH cm:");
            //Serial.println(getRange(ini_Der, end_Der));
            //Serial.print("Medida sIZQUIE cm:");
            //Serial.println(getRange(ini_Izq, end_Izq));
          
            duracionFijada = false;

            
            if (Der > distancia_minima && Der < Front && Der < distancia_localizado){
              duracion = 26; //Giramos durante   0.494 segundos
              Velocidad = MEDIO;
              estadoActual = GirandoDerecha;
            }else if (Izq > distancia_minima && Izq < Front && Izq < distancia_localizado){
              duracion = 26; //Giramos durante 0.494 segundos
              Velocidad = MEDIO;
              estadoActual = GirandoIzquierda;
            }else if (Front < distancia_localizado && Front > distancia_ataque){
               duracion = 53; //Avanzamos durante 1.007 segundos
               Velocidad = MEDIO;
               estadoActual = Avanzando;
            }else if (Front > distancia_minima && Front < distancia_ataque){
               duracion = 53; //Avanzamos durante 1.007 segundos
               Velocidad = RAPIDO;
               estadoActual = Avanzando;
            }else {
                duracion = 30; // Avanzamosjusto el tiempo entre medicion y medicion
                Velocidad = LENTO;
                estadoActual = Avanzando;
            }            

            //Serial.println("Aqui1");
            delay(3000);
            //Serial.println("Aqui2");
          }

           /*Serial.println("iter1");
           
           if (digitalRead(INFRADELANTE) == HIGH){
              Serial.println("DELANTE: HIGH"); 
           }else if (digitalRead(INFRADELANTE) == LOW){
               Serial.println("DELANTE: LOW");
           }

           if (digitalRead(INFRADETRAS) == HIGH){
              Serial.println("DETRAS: HIGH"); 
           }else if (digitalRead(INFRADETRAS) == LOW){
               Serial.println("DETRAS: LOW");
           }

           

           delay(500);*/
          break;
        
        
        
        case BuscandoEnemigo:
          //Serial.println("Buscando Enemigo");
          if (ComprobarLineas(estadoActual)){
            duracionFijada = false;
           
          }else{
            if (!medicionIniciada){
              StartMeasure();
              medicionIniciada = true;
            }

            if (medido_Fre && medido_Der && medido_Izq){
              medicionIniciada = false;
              
              Front = getRange(ini_Fre, end_Fre);
              Der = getRange(ini_Der, end_Der);
              Izq = getRange(ini_Izq, end_Izq);
  
              /*Serial.print("Medida sFRENTE cm:");
              Serial.println(getRange(ini_Fre, end_Fre));
              Serial.print("Medida sDERECH cm:");
              Serial.println(getRange(ini_Der, end_Der));
              Serial.print("Medida sIZQUIE cm:");
              Serial.println(getRange(ini_Izq, end_Izq));
              */
              duracionFijada = false; 
              
              if (Der > distancia_minima && Der < Front && Der < distancia_localizado){
                duracion = 30; //Giramos durante   0.494 segundos
                Velocidad = MEDIO;
                estadoActual = GirandoDerecha;
              }else if (Izq > distancia_minima && Izq < Front && Izq < distancia_localizado){
                duracion = 30; //Giramos durante 0.494 segundos
                Velocidad = MEDIO;
                estadoActual = GirandoIzquierda;
              }else if (Front < distancia_localizado && Front > distancia_ataque){
                 duracion = 53; //Avanzamos durante 1.007 segundos
                 Velocidad = MEDIO;
                 estadoActual = Avanzando;
              }else if (Front > distancia_minima && Front < distancia_ataque){
                 duracion = 53; //Avanzamos durante 1.007 segundos
                 Velocidad = RAPIDO;
                 estadoActual = Avanzando;
              }else {
                  duracion = 30; // Avanzamosjusto el tiempo entre medicion y medicion
                  Velocidad = LENTO;
                  estadoActual = Avanzando;
              }
            }
         } 
          break;



          
        case Avanzando:
        //Serial.print("Avanzando: "); Serial.println(duracion);

         if (ComprobarLineas(estadoActual)){
            duracionFijada = false;
         }else{
            if (!duracionFijada){
              duracionFijada = true;
              analogWrite(PWM, Velocidad);
              avanzar();
            }
  
            if (duracion == 1){
              duracionFijada = false;
              analogWrite(PWM, LENTO);
              estadoActual = BuscandoEnemigo;
            }
           }
          break;




        case GirandoDerecha:
        //Serial.println("Girando Derecha");
           if (ComprobarLineas(estadoActual)){
            duracionFijada = false;
            }else{
              if (!duracionFijada){
                  duracionFijada = true;
                  analogWrite(PWM, Velocidad);
                  girarDerecha();
              }
              
              if (duracion == 1){
                duracionFijada = false;
                analogWrite(PWM, LENTO);
                estadoActual = BuscandoEnemigo;
              }
            }
          break;



        
        case GirandoIzquierda:
        //Serial.println("Girando Izquierda");
           if (ComprobarLineas(estadoActual)){
            duracionFijada = false;
           }else{
              if (!duracionFijada){
                  duracionFijada = true;
                  analogWrite(PWM, Velocidad);
                  girarIzquierda();
              }
              
              if (duracion == 1){
                duracionFijada = false;
                analogWrite(PWM, LENTO);
                estadoActual = BuscandoEnemigo;
              }
           }
          break;



          case HuirLineaDelante:
           //Serial.println("HuirDelante");
            duracionFijada = false;
            duracion = 79; //Avanzamos durante 1.501 seg
            Velocidad = RAPIDO;
            estadoActual = HuyendoMarchaDetras;
          break;

          
          
          case HuirLineaDetras:
            //Serial.println("HuirDelante");
            duracionFijada = false;
            duracion = 79; //Avanzamos durante 1.501 seg
            Velocidad = RAPIDO;
            estadoActual = HuyendoMarchaDelate;

          break;

          
          
          case HuyendoMarchaDetras:
           //Serial.println("HuirMarchaDetras");
              if (!duracionFijada){
                duracionFijada = true;
                analogWrite(PWM, Velocidad);
                retroceder();
              }

              if (duracion == 1){
                duracionFijada = false;
                duracion = 79; //Giramos durante 1.501 seg
                Velocidad = RAPIDO;
                estadoActual = HuyendoGirarDerecha;
              }
          break;
          
          
          
          case HuyendoMarchaDelate:
           //Serial.println("HuirMarchaDelante");
                if (!duracionFijada){
                  duracionFijada = true;
                  analogWrite(PWM, Velocidad);
                  avanzar();  
                }

                if (duracion == 1){
                  duracionFijada = false;
                  duracion = 79; //Giramos durante 1.501 seg
                  Velocidad = RAPIDO;
                  estadoActual = HuyendoGirarDerecha; 
                }
          break;

          
          case HuyendoGirarDerecha:
            if (!duracionFijada){
              duracionFijada = true;
              analogWrite(PWM, Velocidad);
              girarDerecha();  
            }

            if (duracion == 1){
                duracionFijada = false;

                analogWrite(PWM, LENTO);
                estadoActual = BuscandoEnemigo;
            }
             
          break;
          
      } 
    }
}


//INTERRUPCIONES:
//INTERRUPCION TIMER 2
ISR(TIMER2_COMPA_vect){
  ++contador;
  
  if (contador == 0){
    if(duracion > 1)
      --duracion;
    if(duracion==1)
      parar();
  }
}



void ISR_sFRENTE() {
  switch (digitalRead(sFRENTE)) {
    case HIGH:
      ini_Fre = micros();
      break;
    case LOW:
      end_Fre = micros();
      medido_Fre = true;
      break;
  }
}


void ISR_sIZQUIE() {
  switch (digitalRead(sIZQUIE)) {
    case HIGH:
      ini_Izq = micros();
      break;
    case LOW:
      end_Izq = micros();
      medido_Izq = true;
      break;
  }
}


void ISR_sDERECH() {
  switch (digitalRead(sDERECH)) {
    case HIGH:
      ini_Der = micros();
      break;
    case LOW:
      end_Der = micros();
      medido_Der = true;
      break;
  }
}




