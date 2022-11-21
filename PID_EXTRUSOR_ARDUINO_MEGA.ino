//----------Termistor---------------------------------------
#include <thermistor.h>

thermistor therm1(A1,0);    //Añadimos el termistor1, en el pin A1, y en la libreria es el TERMISTOR_0
thermistor therm2(A2,1);    //Añadimos el termistor2, en el pin A2, y en la libreria es el TERMISTOR_1

unsigned long currentTime1, previousTime1;
double elapsedTime1;
double error1, lastError1, cumError1, rateError1;
double Input1;

unsigned long currentTime2, previousTime2;
double elapsedTime2;
double error2, lastError2, cumError2, rateError2;
double Input2;

double kp = 10;    //Definimos kp
double ki = 0;    //Definimos ki
double kd = 0;    //Definimos kd
double Setpoint = 225;  //Definimos setpoint
double temp1;
double temp2;

int periodo = 1000;             //Tomara la temperatura cada segundo
unsigned long TiempoAhora = 0;  //Guardara la Temperatura usando millis()

//----------------------------------------------------------

//------------------Calentadores----------------------------
int PWM_Calentador_1, PWM_Calentador_2;
int Calentador_1 = 6;
int Calentador_2 = 7;


void setup() {
  Serial.begin(9600);

  //Definimos las salidas
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop() {
 
  
  //------------------------Medicion de Temperatura-----------------------------

  temp1 = therm1.analog2temp(); //Leemos las temperaturas
  //temp2 = therm2.analog2temp(); //En so de leer otra temperatura
  
  Serial.print(temp1);
  Serial.print(",");
  //Serial.print(temp2);
  //Serial.print(",");   
  
  //---------------------------------------------------------------------------

  //------------------------Calentamiento de Resistencias----------------------
  
  PWM_Calentador_1 = computePID1(temp1);
  //PWM_Calentador_2 = computePID2(temp2);

  //----------------------- Limites Calentador 1-------------------------------
  if(PWM_Calentador_1 > 255){
    analogWrite(Calentador_1, 255);
    Serial.print(255);
    Serial.print(",");
  }else{
    analogWrite(Calentador_1, PWM_Calentador_1);
    Serial.print(PWM_Calentador_1);
    Serial.print(",");
  }

  /*
  //-------------------------Limites Calentador 2------------------------------
  if(PWM_Calentador_2 > 255){
    analogWrite(Calentador_2, 255);
    Serial.print(255);
    Serial.print(",");
  }else{
    analogWrite(Calentador_2, PWM_Calentador_2);
    Serial.print(PWM_Calentador_2);
    Serial.print(",");
  }

  //----------------------------------------------------------------------------
  */
  Serial.println("");
  /*
  analogWrite(Calentador_1, PWM_Calentador_1);
  analogWrite(Calentador_2, PWM_Calentador_2);
  */
  /*
  Serial.print(PWM_Calentador_1);
  Serial.print(",");
  Serial.println(PWM_Calentador_2);
  */
  
  
  //---------------------------------------------------------------------------
  
  delay(2000);
}

double computePID1(double Input){     
        currentTime1 = millis();                               // obtener el tiempo actual
        elapsedTime1 = (double)(currentTime1 - previousTime1);     // calcular el tiempo transcurrido

        
        error1 = Setpoint - Input1;                               // determinar el error entre la consigna y la medición
        cumError1 += error1 * elapsedTime1;                      // calcular la integral del error
        rateError1 = (error1 - lastError1) / elapsedTime1;         // calcular la derivada del error
 
        double output1 = kp*error1 + ki*cumError1 + kd*rateError1;     // calcular la salida del PID
 
        lastError1 = error1;                                      // almacenar error anterior
        previousTime1 = currentTime1;                             // almacenar el tiempo anterior
 
        return output1;
}

double computePID2(double Input){     
        currentTime2 = millis();                               // obtener el tiempo actual
        elapsedTime2 = (double)(currentTime2 - previousTime2);     // calcular el tiempo transcurrido

        
        error2 = Setpoint - Input2;                               // determinar el error entre la consigna y la medición
        cumError2 += error2 * elapsedTime2;                      // calcular la integral del error
        rateError2 = (error2 - lastError2) / elapsedTime2;         // calcular la derivada del error
 
        double output2 = kp*error2 + ki*cumError2 + kd*rateError2;     // calcular la salida del PID
 
        lastError2 = error2;                                      // almacenar error anterior
        previousTime2 = currentTime2;                             // almacenar el tiempo anterior
 
        return output2;
}
