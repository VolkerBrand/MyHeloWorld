


// #define SUPPORT_XCELL_RT    //uncomment if X-CELL RT connected. FC1.4: pas_factor_min=0.2, pas_factor_max=0.5. FC1.5: pas_factor_min=0.5, pas_factor_max=1.5. pas_magnets=8
// #define SUPPORT_SEMPU_V1    //uncomment if you have a Sempu torque sensor, old type with one direction wire and one pas wire
// #define SUPPORT_SEMPU_T4_3A //uncomment if you have a Sempu torque sensor, of type left&right crank torque 1.5V-3V(4V), one speed signal, one direction/pedaling signal, 32 pulses per revolution
// #define SUPPORT_SEMPU_T4_3B //uncomment if you have a Sempu torque sensor, of type left&right crank torque 1.5V-3V(4V), dual speed/direction signal cos/sin 2x16 pulses per revolution
// #define SUPPORT_SEMPU_ADVANCED  //uncomment if you have a Sempu torque sensor with individal Parameter Settings
const float torque_average_count = 1;  //default 1=full crank turn; Values less 1 as percent of crank turn: 0.5=half turn, 0.25=quarter turn, Values greater than 1 set the average time fix to Milliseconds 1.1=1.1ms average time 
const float torque_sensitivity = 10; // Sempu torque sensor sensitivity given in mV/Nm
// #define SUPPORT_TORQUE_THROTTLE
// #define TORQUE_PAS_DIRECTION //uncomment if you have a Sempu torque sensor with direction signal
// #define TORQUE_BOTH_SITES //uncomment if you have a Sempu torque sensor with real signal for the left and right crank
