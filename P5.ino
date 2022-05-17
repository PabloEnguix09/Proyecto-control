/* 
PRACTICA 1: IMPLEMENTACION DE UN CONTROLADOR PARA UN MOTOR DE DC

*/

// DECLARACIONES /////1/////////////////////////////////////////////////////////////////////

// ACTIVACION DE CODIGO

#define NOMBRE_PRAC "P1-C"
#define VERSION_SW "1.0"

#define ACTIVA_P1A
//#define DEBUG_P1A
#define ACTIVA_P1B1
#define ACTIVA_P1B2
#define ACTIVA_P1B3
#define ACTIVA_P1C
#define DEBUG_P1C
// #define ACTIVA_P1C_MED_ANG
//#define ACTIVA_P1D2
#define ACTIVA_P1D3

// Display OLED ///////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

// Parametros cola de la interrupcion del encoder ///////////////////////////////////////
#define TAM_COLA_I 1024 /*num mensajes*/
#define TAM_MSG_I 1 /*num caracteres por mensaje*/

// TIEMPOS
#define BLOQUEO_TAREA_LOOPCONTR_MS 10 
#define BLOQUEO_TAREA_MEDIDA_MS 10

// Configuración PWM  ////////////////////////////////////////////////////////////////////
uint32_t pwmfreq = 1000; // 1KHz
const uint8_t pwmChannel = 0;
const uint8_t pwmresolution = 8;
const int PWM_Max = pow(2,pwmresolution)-1; //

// Pines driver motor ////////////////////////////////////////////////////////////////////
const uint8_t PWM_Pin = 32; // Entrada EN 
const uint8_t PWM_f = 16; // Entrada PWM1 
const uint8_t PWM_r = 17; // Entrada PWM2 

// Voltaje maximo motor ////////////////////////////////////////////////////////////////////
float SupplyVolt = 12;

// Pines encoder ////////////////////////////////////////////////////////////////////
const uint8_t A_enc_pin = 35;
const uint8_t B_enc_pin = 34;

// Conversión a angulo y velocidad del Pololu 3072
//const float conv_rad = ; 
//const float conv_rev = ;
//const float conv_rad_grados = ; 

// Declarar funciones ////////////////////////////////////////////////////////////////////
void config_sp(); // Configuracion puerto serie
void config_oled(); // Configuracion OLED
void config_enc(); // Configuracion del encoder
void config_PWM(); // Configuracion PWM
void excita_motor(float v_motor); // Excitacion motor con PWM
//float interpola_vel_vol_lut(float x); // Interpolacion velocidad/voltios LUT

// TABLA VELOCIDAD-VOLTAJE P1D
#ifdef ACTIVA_P1D2
#define LONG_LUT 12
//Vector de tensiones
const float Vol_LUT[LONG_LUT] = {0, 1, 1.6, 2, 3, 4, 5, 6, 7, 8, 9, 100};
// Vector de velocidades
const float Vel_LUT[LONG_LUT] = {0, 0, 0.7, 2.08, 4.5, 6.5, 7.2, 8, 8.5, 8.9, 9.1, 9.5};
#endif

// Variables globales ////////////////////////////////////////////////////////////////////
int32_t ang_cnt = 0;
float pwm_volt = 0;
float pwm_anterior = 0;
int32_t pwm_motor = 0;
//int32_t sign_v_ant = 0;
float v_medida = 0;    // Valor medido de angulo o velocidad -----------------
float v_medida_ant = 0;
float ref_val = 0;     // Valor de referencia de angulo o velocidad
int8_t start_stop = 0; //1 -> en funcionamiento | 0 -> parado 
float K_p = 6;
float K_i = 12;
float K_d = 0.0005;

float vi_ant = 0;
float e_ant = 0;

// Declaracion objetos  ////////////////////////////////////////////////////////////////////

xQueueHandle cola_enc; // Cola encoder

/*
 RUTINAS ATENCION INTERRUPCIONES ########################################################################
*/

/* 
 Rutina de atención a interrupción ISC_enc --------------------------------------------
*/

void IRAM_ATTR ISR_enc() {
	// Lee las salidas del Encoder		
	int pinA = digitalRead(A_enc_pin);
  int pinB = digitalRead(B_enc_pin);
  
	// Procesa los datos
  uint8_t nDec = -1;
  
  nDec = 2*pinA + pinB;

	// Enviar los bytes a la cola 
  //se pone el ampersand para apuntar a la direccion de memoria,
  //si pusiesemos asterisco nos da el valor que guardamos en la direccion de memoria
	if (xQueueSendFromISR(cola_enc, &nDec ,NULL) != pdTRUE)
	{
	  printf("Error de escritura en la cola cola_enc \n");
	};
}

/*
 TAREAS #############################################################################
*/

/*
 Tarea task_enc #####################################################################
*/
#ifdef ACTIVA_P1A
void task_enc(void* arg) {
	// Declaracion de variables locales
  uint8_t dato;
  uint8_t datoA = 0; 
  
	while(1){
		// Espera a leer los datos de la cola
		if (xQueueReceive(cola_enc , &dato,(TickType_t) portMAX_DELAY) == pdTRUE){
			// Codificar la fase del encoder
      if(dato==0) dato = 1;
      else if(dato==1) dato = 2;
      else if(dato==2) dato = 4;
      else if(dato==3) dato = 3;
      
			// Calcular incremento/decremento y actualizar valor 
      if(datoA<dato &&  (datoA-dato) == -1 || (datoA-dato) == 3) ang_cnt++;
      else ang_cnt--;
      
      datoA = dato;
      
			#ifdef DEBUG_P1A
				// Enviar al monitor serie
        //esto no permite encadenar informacion, hay que hacer varios serials
        Serial.print("Fase: ");
        Serial.println(dato);
        Serial.print("Ang cont: ");
        Serial.println(ang_cnt);
			#endif
		 } else {
		 	printf("Error de lectura de la cola cola_enc \n");
		 }
	}
}
#endif


/* 
Tarea de configuración de parámetros  #####################################################################
*/
void task_config(void *pvParameter) {
	char ini_char = '0';

	while(1) { 
		// Detectar caracter enviado
    if(Serial.available()>0){
      ini_char = Serial.read(); 
      if(ini_char == 'V') {
        // Guardar valor recibido
       pwm_volt = Serial.parseFloat();
        // Escribir el valor recibido en la consola
        Serial.print("Voltaje motor= ");
        Serial.println(pwm_volt);
      }

      if(ini_char == 'R') {
        // Guardar valor recibido
        ref_val = Serial.parseFloat();
       // Escribir el valor recibido en la consola
        Serial.print("Valor de refenrecia = ");
        Serial.print(ref_val);
        #ifdef ACTIVA_P1C_MED_ANG // Medida de angulo
         Serial.println("º");
        #else // Medida de velocidad
         Serial.println(" rps");
       #endif
      }

      if(ini_char == 'S') {
        // Guardar valor recibido
       start_stop = Serial.parseInt();
       if (start_stop == 1) Serial.println("-- START --");
       else Serial.println("-- STOP --");
      }


      if(ini_char == 'P') {
        // Guardar valor recibido
        K_p = Serial.parseFloat();
        // Escribir el valor recibido en la consola
      }

      if(ini_char == 'I') {
        // Guardar valor recibido
        K_i = Serial.parseFloat();
        // Escribir el valor recibido en la consola
      }

      if(ini_char == 'D') {
        // Guardar valor recibido
        K_d = Serial.parseFloat();
        // Escribir el valor recibido en la consola
      }
      
    }
		// Activacion de la tarea cada 0.1s
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef ACTIVA_P1B3
void task_loopcontr(void* arg) { //TODO: Empezar aquí, poner P, luego PI, luego PID

	while(1) {
    if(start_stop==1){
      v_medida = 2*PI*ang_cnt / 1200;
      
      #ifndef ACTIVA_P1C_MED_ANG
        float v_medida_rad = 2*PI*ang_cnt / 1200;
        v_medida = ((v_medida_ant- v_medida_rad)/(2*PI))/0.01;
      #endif
      v_medida_ant = v_medida_rad;
      #ifdef ACTIVA_P1D2
        pwm_volt = interpola_vel_vol_lut(ref_val);
      #endif
      #ifdef ACTIVA_P1D3
        float error = ref_val - v_medida;
        float Tm = BLOQUEO_TAREA_LOOPCONTR_MS/1000.0;
        float vp = K_p*error;
        float vi = vi_ant + K_i*Tm*error;
        float vd = (K_d/Tm) * (error-e_ant);
        pwm_volt = vp + vi + vd;
      
        e_ant = error;
        vi_ant = vi;
      #endif
    	// Excitacion del motor con PWM
  		excita_motor(pwm_volt);
    }
    else {
      pwm_motor = 0;
      excita_motor(0);
      ang_cnt = 0;
      v_medida = 0;
   }
		// Activacion de la tarea cada 0.01s
	  vTaskDelay(BLOQUEO_TAREA_LOOPCONTR_MS/portTICK_RATE_MS);
	}

}
#endif

/* 
Tarea del lazo principal del controlador  #####################################################################
*/
#ifdef DEBUG_P1C
void task_medidas(void* arg) 
{

	while(1) {
    if(start_stop==1){
  		// Mostrar medidas de angulo y velocidad del motor
  		#ifdef ACTIVA_P1C_MED_ANG // Medida de angulo
        v_medida = v_medida * 360 / (2*PI);
        Serial.print("Med: ");
        Serial.print(v_medida);
  		#else // Medida de velocidad
        Serial.print("Med: ");
        Serial.print(v_medida);
  		#endif

      Serial.print(",  Ref: ");
      Serial.println(ref_val);
  	}
		// Activacion de la tarea cada 1s
    vTaskDelay(BLOQUEO_TAREA_MEDIDA_MS/portTICK_RATE_MS);
	}
}
#endif

/*
SET UP -----------------------------------------------------------------------------------
*/
void setup() {
	// Configuracion puerto serie
	config_sp();
	
	// Configuracion OLED
	config_oled();

	// Configuracion PWM
  config_PWM();
	// Crear cola_enc
  cola_enc = xQueueCreate(TAM_COLA_I, TAM_MSG_I);
	if(cola_enc == NULL){
	  Serial.println("Error en creacion de cola_enc");
	 	exit(-1);
  };

	// Crear la tarea task_enc
	if(xTaskCreate( task_enc, "task_enc", 2048, NULL, 1, NULL) != pdPASS){
	 	Serial.println("Error en creacion tarea task_enc");
	 	exit(-1);
	}

	// Crear la tarea task_config
  xTaskCreate(task_config, "task_config", 2048, NULL, 1, NULL);

	// Crear la tarea task_loopcontr
  xTaskCreate(task_loopcontr, "task_loopcontr", 2048, NULL, 1, NULL);

	#ifdef DEBUG_P1C
		// Crear la tarea task_medidas
    xTaskCreate(task_medidas, "task_medidas", 2048, NULL, 1, NULL);
	#endif

	// Configuracion del encoder
  config_enc();
}

/*
LOOP ---- NO USAR ------------------------------------------------------------------- 
*/
void loop() {}

// FUNCIONES ////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del encoder
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1A
void config_enc(){
	// Configuracion de pines del encoder
  pinMode(A_enc_pin, INPUT);
  pinMode(B_enc_pin, INPUT);
	// Configuracion interrupcion
  attachInterrupt(digitalPinToInterrupt(A_enc_pin), ISR_enc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(B_enc_pin), ISR_enc, CHANGE);
} 
#endif
////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B2
void config_PWM(){
	// Configuracion de pines de control PWM
  pinMode(PWM_f, OUTPUT);
  pinMode(PWM_r, OUTPUT);
	// Configuracion LED PWM 
  ledcSetup(pwmChannel,pwmfreq, pwmresolution);
	// Asignar el controlador PWM al GPIO
  ledcAttachPin(PWM_Pin,pwmChannel);
}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion excitacion del motor con PWM
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1B3
void excita_motor(float v_motor){
  if(v_motor>0 && pwm_anterior < 0){
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, LOW);
  }
  if(v_motor < 0 && pwm_anterior > 0) {
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, LOW);
  }
	// Sentido de giro del motor
  if(v_motor>0){
    digitalWrite(PWM_f, LOW);
    digitalWrite(PWM_r, HIGH);
  } else {
    digitalWrite(PWM_r, LOW);
    digitalWrite(PWM_f, HIGH);
  }

  pwm_anterior = v_motor;
	// Calcula y limita el valor de configuración del PWM
  if(v_motor < 0) v_motor = -v_motor;
  if(v_motor>SupplyVolt) v_motor = SupplyVolt;
	// El valor de excitación debe estar entro 0 y PWM_Max
  int excitacion = trunc(v_motor * PWM_Max / SupplyVolt);
	// Excitacion del motor con PWM
	ledcWrite(pwmChannel, excitacion);
}  
#endif

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del puerto serie
////////////////////////////////////////////////////////////////////////////////////
void config_sp(){
	Serial.begin(115200);
	Serial.println("  ");
	Serial.println("--------------------------------------------");
	Serial.println("PRACTICA CONTROLADOR MOTOR " NOMBRE_PRAC);
	Serial.println("--------------------------------------------");
}  

////////////////////////////////////////////////////////////////////////////////////
// Funcion configuracion del OLED
////////////////////////////////////////////////////////////////////////////////////
void config_oled(){
	Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ;
	display.clearDisplay();
	display.setTextColor(WHITE);        // 
	display.setCursor(0,0);             // Start at top-left corner
	display.println(F("CONTR. MOTOR " NOMBRE_PRAC));
	display.display();
	delay(1000);
	display.setTextColor(BLACK,WHITE);        // 
	display.setCursor(0,20);             // Start at top-left corner
	display.println(F(" SW v." VERSION_SW));
	display.display();
	delay(1000);
}  


////////////////////////////////////////////////////////////////////////////////////
// Funcion de interpolacion LUT de Velocidad-Voltaje
////////////////////////////////////////////////////////////////////////////////////
#ifdef ACTIVA_P1D2
float interpola_vel_vol_lut(float x) {
	// Buscar el valor superior más pequeño del array
	int8_t i = 0;
	if ( x >= Vel_LUT[LONG_LUT - 2] ) {i = LONG_LUT - 2;}
	else {while ( x > Vel_LUT[i+1] ) i++;}

	// Guardar valor superior e inferior
	float xL = Vel_LUT[i];
	float yL = Vol_LUT[i];
	float xR = Vel_LUT[i+1];
	float yR = Vol_LUT[i+1];

	// Interpolar
	float dydx = ( yR - yL ) / ( xR - xL );

	return yL + dydx * ( x - xL );
}
#endif
