/*
SEMPU:    eigentlich SEMPU_NEW beidseitige Messung mit zwei Speed Signale 
SEMPU_V1: erste Sempu Generation, Messung nur Links, mit ein speed Signal plus ein DIR signal
Fragen im Code mit <-- markiert:

-1. Drehmoment wird zwei mal gemessen, einmal instant, einmal ins array, Array wird einfach rotierend voll geschrieben
-2: power currently set by throttle <-- diese Zeile wird immer im loop ausgefuehrt!?


NEU:
	Torque Sensor SUPPORT_SEMPU renamed to SUPPORT_SEMPU_T4_3B and add new Sensor SUPPORT_SEMPU_T4_3A
	
*/

//zeile 91-93:
#if defined(SUPPORT_LIGHTS_SWITCH) && (defined(SUPPORT_XCELL_RT)||defined(SUPPORT_SEMPU_T4_3A)||defined(SUPPORT_SEMPU_T4_3B)||defined(SUPPORT_SEMPU_V1)) && HARDWARE_REV < 20
#error Software controlled lights switch is not compatible with torque sensor support on FC < 2.0
#endif

//zeile 136-137:
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B) 
    float wh_human; //human watthours
#endif


// zeile 257 bis 261
#if defined(SUPPORT_SEMPU_V1)
	#define pas_time 1875 //conversion factor for pas_time to rpm 32 pulses per revolution (cadence) (60000ms/32=1875ms)
#elif defined(SUPPORT_SEMPU_T4_3A)
	#define pas_time 1875 //32 speed pulses per revolution <--!!!! must be doublechecked !!!! 60000ms/32=1875ms conversion factor for pas_time to rpm (cadence)
#elif defined(SUPPORT_SEMPU_T4_3B)
	#define pas_time 2500 //24 speed pulses per revolution 60000ms/24=2500ms conversion factor for pas_time to rpm (cadence)
#else
  #define pas_time 60000/pas_magnets //conversion factor for pas_time to rpm (cadence)
#endif

//zeile 274-290
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
int torque_zero=TORQUE_ZERO; //Offset of torque sensor. Adjusted at startup if TORQUE_AUTOZERO option is set
static volatile boolean analogRead_in_use = false; //read torque values in interrupt only if no analogRead in process
static volatile boolean torque_want_calculation = false; //read torque values in interrupt only if no analogRead in process
#if HARDWARE_REV<20 && defined(SUPPORT_XCELL_RT)
const int torquevalues_count=8;
volatile int torquevalues[torquevalues_count]= {0,0,0,0,0,0,0,0}; //stores the 8 torque values per pedal roundtrip
#elif defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_XCELL_RT)
const int torquevalues_count=16;
volatile int torquevalues[torquevalues_count]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //stores the 16 torque values per pedal roundtrip (Thun) or half roundtrip (Sempu)
#elif defined(SUPPORT_SEMPU_T4_3B)
const int torquevalues_count=24;
volatile int torquevalues[torquevalues_count]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //stores the 24 torque values per pedal roundtrip
#elif defined(SUPPORT_SEMPU_T4_3A)  
const int torquevalues_count=32;
volatile int torquevalues[torquevalues_count]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //stores the 32 torque values per pedal roundtrip 
#endif
volatile byte torqueindex=0;        //index to write next torque value
volatile boolean readtorque=false;  //true if torque array has been updated -> recalculate in main loop
#endif

//zeile 488-500
#if !defined(SUPPORT_XCELL_RT) && !defined(SUPPORT_BBS) && !defined(SUPPORT_SEMPU_T4_3B)
    bitSet(EICRB,2);      //trigger on any edge INT5 for PAS sensor
    EIMSK  |= (1<<INT5);  //turn on interrupt INT5 for PAS sensor
#else                                                                                         // else we have sensors with two signals (cos/sin) sensors
    bitClear(DDRE,6);      //configure PE6 as input
    bitSet(PORTE,6);       //enable pull-up on PAS 2 sensor
    bitSet(EICRB,2);      //trigger on rising edge INT5 for Thun sensor/BBS/Sempu
    bitSet(EICRB,3);      //trigger on rising edge INT5 for Thun sensor/BBS/Sempu
    bitSet(EICRB,4);      //trigger on rising edge INT6 for Thun sensor/BBS/Sempu
    bitSet(EICRB,5);      //trigger on rising edge INT6 for Thun sensor/BBS/Sempu
    EIMSK  |= (1<<INT5);  //turn on interrupt INT5 for PAS sensor
    EIMSK  |= (1<<INT6);  //turn on interrupt for Thun sensor/BBS/Sempu
#endif

//zeile 530-534
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
#ifdef TORQUE_AUTOZERO
    torque_rezero();
#endif
#endif



// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! alles ab zeile 574 ist innerhalb void loop()




//zeile 668-699
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1)  || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
#ifdef SUPPORT_XCELL_RT
      torque_instant=0.49*(analogRead(option_pin)-torque_zero);  //multiplication constant for THUN X-CELL RT is approx. 0.49Nm/count
#else //any Sempu
      torque_instant=0.33*(analogRead(option_pin)-torque_zero);  //multiplication constant for any SEMPUs are approx. 0.33Nm/count
#endif
    if (readtorque==true)
    {
        torque=0.0;
        for (int i = 0; i < torquevalues_count; i++)
        {
            torque+=torquevalues[i];
        }
        readtorque=false;
		//now calculate torque_instant, torque and power
		#ifdef SUPPORT_XCELL_RT
		#if HARDWARE_REV<20
				torque=abs((torque)*0.061);   //torque=sum of torque values/#of torque values*5V/1023 counts/(10mV/Nm)
		#else
				torque=abs((torque)*0.03054740957);
		#endif
		power_human=0.20943951*cad*torque;   //power=2*pi*cadence*torque/60s*2 (*2 because only left side torque is measured by x-cell rt)
		#elif defined(SUPPORT_SEMPU_V1) //SEMPU V1
			torque=abs((torque)*0.02078055073);   // torque=sum of torque values/#of torque values(16)*5V/1023 counts/(14.7 mV/Nm)
			power_human=0.10471975512*cad*torque;   //power=2*pi*cadence*torque/60s
		#elif defined(SUPPORT_SEMPU_T4_3A) 
			torque=abs((torque)*0.01039027537);   // torque=sum of torque values/#of torque values(32) *5V/1023 counts/(14.7 mV/Nm) <-- to CHECK #of values!!!!!!!!!!!!!!!!
			power_human=0.10471975512*cad*torque;   //power=2*pi*cadence*torque/60s
			#else // SEMPU_T4_3B
			torque=abs((torque)*0.01385370049);   // torque=sum of torque values/#of torque values(24) *5V/1023 counts/(14.7 mV/Nm)
			power_human=0.10471975512*cad*torque;   //power=2*pi*cadence*torque/60s
		#endif
    }
#endif

//zeile 730-732
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
            wh_human=variable.wh_human;
#endif


// zeile 786 bis 805 
//Power control-------------------------------------------------------------------------------------------------------------
#ifdef SUPPORT_BBS
   if ((millis()-bbs_pausestart)<BBS_GEARCHANGEPAUSE)  //activate brake for specified pause time to allow for gear change
     brake_stat=0;
#endif
    power_throttle = throttle_stat / 1023.0 * curr_power_max;         // <--2. power currently set by throttle

#if CONTROL_MODE == CONTROL_MODE_TORQUE                      //human power control mode
	#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
		power_poti = poti_stat/102300.0* curr_power_poti_max*power_human*(1+spd/20.0); //power_poti_max is in this control mode interpreted as percentage. Example: power_poti_max=200 means; motor power = 200% of human power
		#ifdef SUPPORT_TORQUE_THROTTLE                              //we want to trigger throttle just by pedal torque
			if (abs(torque_instant)>torque_throttle_min)            //we are above the threshold to trigger throttle
			{
			double power_torque_throttle = abs(torque_instant/torque_throttle_full*poti_stat/1023*curr_power_max);  //translate torque_throttle_full to maximum power
			power_throttle = max(power_throttle,power_torque_throttle); //decide if thumb throttle or torque throttle are higher
			power_throttle = constrain(power_throttle,0,curr_power_max); //constrain throttle value to maximum power 
			}
		#endif
	#endif
#endif

//  !!!!!!!!!!!!!!! Ab Zeile 1075 sind wir unter der void loop() es folgen Deklarationen die in Funktionen benutzt werden und Funktionen


//zeile 1075 1097 
#if HARDWARE_REV >= 20 //attach interrupts manually
ISR(INT7_vect)
{
    speed_change();
}
#ifdef SUPPORT_PAS
	#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_BBS) || defined(SUPPORT_SEMPU_T4_3B)  //Sensors with two speed signals (cos/sin)
		ISR(INT5_vect)
		{
			pas_change_dual(false);
		}
		ISR(INT6_vect)
		{
			pas_change_dual(true);
		}
	#else // Sensors with one speed signal 
		ISR(INT5_vect)
		{
			pas_change();
		}
	#endif
	#endif
#endif


//zeile 1099 bis 1179
#if HARDWARE_REV >= 20
    #if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_BBS) || defined(SUPPORT_SEMPU_T4_3B)   
		void pas_change_dual(boolean signal)
		{
			if (signal)
				pedaling=bitRead(PINE,5);
			else
			{
				pedaling=!bitRead(PINE,6);
				#ifdef SUPPORT_XCELL_RT
					cad=7500/(millis()-last_pas_event); //8 pulses per revolution
				#else                                   //Sempu or BBS with 24 Pulses per Revolution 
					cad=2500/(millis()-last_pas_event); //24 pulses per revolution (60000ms/24=2500ms)
				#endif        
				last_pas_event = millis();
			}
			pedalingbackwards=!pedaling;
			#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_T4_3B)
				if (analogRead_in_use)
				{
					torque_want_calculation = true;
					return;
				}
				read_current_torque();
			#endif
		}
	#endif
#endif

#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
void read_current_torque() //this reads the current torque value
{
    torquevalues[torqueindex]=analogRead(option_pin)-torque_zero;
    torqueindex++;
    if (torqueindex==torquevalues_count)
        torqueindex=0;
    readtorque=true;
}

void torque_rezero() //resets torque sensor
{
  torque_zero=analogRead_noISR(option_pin);
}
#endif

#ifdef SUPPORT_PAS
void pas_change()     //Are we pedaling? PAS Sensor Change-------------------------------------
{
    if (last_pas_event>(millis()-10)) return;
    boolean pas_stat=digitalRead(pas_in);
    if (pas_stat)
    {
        pas_off_time=millis()-last_pas_event;
		#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
        if (analogRead_in_use)
          torque_want_calculation = true;
        else
          read_current_torque();
		#endif
    }
    else
    {pas_on_time=millis()-last_pas_event;}
    last_pas_event = millis();
    cad=pas_time/(pas_on_time+pas_off_time);
    
    #if defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A)
		pedaling=bitRead(PINE,6); //read direction pin of pas sensor and set pedaling to true
    #else
      pas_failtime=pas_failtime+1;
      double pas_factor=(double)pas_on_time/(double)pas_off_time;
      if ((pas_factor>pas_factor_min)&&(pas_factor<pas_factor_max))
      {
          pedaling=true;
          pas_failtime=0;
      }
    #endif
}
#else
#warning PAS sensor support is required for legal operation of a Pedelec  by EU-wide laws except Austria or Swiss.
#endif

//zeile 1284-1289
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
    localSerial->print(MY_F(" Pedaling"));
    localSerial->print(pedaling);
    localSerial->print(MY_F(" Torque"));
    localSerial->print(torque_instant,1); //print current torque value in Nm
#else
	
//zeile 1340-1342
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
    localSerial->print(pedaling); Serial.print(MY_F(";"));
#else
	
//zeile 1525-1527
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
  variable_new.wh_human=wh_human; //save human watthours
#endif

//zeile 1594-1609
int analogRead_noISR(uint8_t pin) //this function makes sure that analogRead is never used in interrupt. only important for X-Cell RT bottom brackets
{
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
    analogRead_in_use = true; //this prevents analogReads in Interrupt from Thun bracket
    if (torque_want_calculation)
    {
      torque_want_calculation=false;
      read_current_torque();
    }
#endif
    int temp=analogRead(pin);
#if defined(SUPPORT_XCELL_RT) || defined(SUPPORT_SEMPU_V1) || defined(SUPPORT_SEMPU_T4_3A) || defined(SUPPORT_SEMPU_T4_3B)
    analogRead_in_use = false;
#endif
    return temp;
}