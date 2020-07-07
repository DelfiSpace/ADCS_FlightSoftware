/*
 * AlgorithmService.cpp
 *
 *  Created on: 17 Jun 2020
 *      Author: Casper, Bart
 */

#include "AlgorithmService.h"


//FRAME:    DESTIONATION SIZE SOURCE <MESSAGE> CRC0 CRC1
//MESSAGE:  SERVICE_NUMBER REQ/REP <Payload>

//Message: Service Req  COMMAND  <Data>;
//command: 0, return desired dipole moment and tumble parameter;
//command: 1, return input data (I used this to verify that the communication with satellite was working correctly and the conversion between bytes and doubles worked)
//command: x, return 0;
bool AlgorithmService::process(DataMessage &command, DataMessage &workingBuffer)
{
    if (command.getService() == ALGORITHM_SERVICE)
     {
        Console::log("Algorithm Service!");

        // prepare response frame
        workingBuffer.setService(ALGORITHM_SERVICE);
        workingBuffer.setMessageType(SERVICE_RESPONSE_REPLY);

        if(command.getDataPayload()[0] == 0){

            workingBuffer.setPayloadSize(48); //three doubles in vector, two vectors returned, so 48 bytes total

            // convert input bytes to doubles
            input2doubles(&command.getDataPayload()[1], mag1Data.x, mag1Data.y, mag1Data.z, mag2Data.x, mag2Data.y, mag2Data.z);

            // run control loop, get desired dipole moment
            vector_t M_DES = this->controlLoop(mag1Data, mag2Data, &s_on, &t_on, &p_tumb, &c_tumb, &c_detumb);

            // convert output to bytes
            for(int i = 0; i < 8; i++){
                calcBuf[i] = ((uint8_t*)&M_DES.x)[i];
            }
            for(int i = 8; i < 16; i++){
                calcBuf[i] = ((uint8_t*)&M_DES.y)[i-8];
            }
            for(int i = 16; i < 24; i++){
                calcBuf[i] = ((uint8_t*)&M_DES.z)[i-16];
            }
            for(int i = 24; i < 32; i++){
                calcBuf[i] = ((uint8_t*)&p_tumb.x)[i-24];
            }
            for(int i = 32; i < 40; i++){
                calcBuf[i] = ((uint8_t*)&p_tumb.y)[i-32];
            }
            for(int i = 40; i < 48; i++){
                calcBuf[i] = ((uint8_t*)&p_tumb.z)[i-40];
            }

            // return output [M_DES.x M_DES.y M_DES.z p_tumb.x p_tumb.y p_tumb.z]
            for(int i = 0; i < 48; i++){
                workingBuffer.getDataPayload()[i] = calcBuf[i];
            }

        }else if(command.getDataPayload()[0] == 1){
            workingBuffer.setPayloadSize(48); //three doubles in vector, two vectors returned, so 48 bytes total

            // convert input bytes to doubles
            input2doubles(&command.getDataPayload()[1], mag1Data.x, mag1Data.y, mag1Data.z, mag2Data.x, mag2Data.y, mag2Data.z);

            // convert doubles to bytes for output
            for(int i = 0; i < 8; i++){
                calcBuf[i] = ((uint8_t*)&mag1Data.x)[i];
            }
            for(int i = 8; i < 16; i++){
                calcBuf[i] = ((uint8_t*)&mag1Data.y)[i-8];
            }
            for(int i = 16; i < 24; i++){
                calcBuf[i] = ((uint8_t*)&mag1Data.z)[i-16];
            }
            for(int i = 24; i < 32; i++){
                calcBuf[i] = ((uint8_t*)&mag2Data.x)[i-24];
            }
            for(int i = 32; i < 40; i++){
                calcBuf[i] = ((uint8_t*)&mag2Data.y)[i-32];
            }
            for(int i = 40; i < 48; i++){
                calcBuf[i] = ((uint8_t*)&mag2Data.z)[i-40];
            }

            // return output [mag1Data.x mag1Data.y mag1Data.z mag2Data.x mag2Data.y mag2Data.z]
            for(int i = 0; i < 48; i++){
                workingBuffer.getDataPayload()[i] = calcBuf[i];
            }

        }
        else{
            //return 0
            workingBuffer.setPayloadSize(1);
            workingBuffer.getDataPayload()[0] = 0;
        }

        //command processed
        return true;
     }
     return false;
}


// DETUMBLE ALGO FUNCTIONS
unsigned int AlgorithmService::c_tumb = 0;
unsigned int AlgorithmService::c_detumb = 0;

/*
 * Description: converts bytes to doubles
 * input:  byte array, the 6 doubles to which the bytes should be assigned to
 * output: doubles with magnetic field reading values
 */
void AlgorithmService::input2doubles(uint8_t a[], double& vx1, double& vy1, double& vz1, double& vx2, double& vy2, double& vz2){


    int itcntr = 0; //iteration counter

    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vx1)[i-itcntr]= a[i];
    }

    itcntr += 8;
    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vy1)[i-itcntr]= a[i];
    }

    itcntr += 8;
    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vz1)[i-itcntr]= a[i];
    }

    itcntr += 8;
    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vx2)[i-itcntr]= a[i];
    }

    itcntr += 8;
    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vy2)[i-itcntr]= a[i];
    }

    itcntr += 8;
    for(int i = itcntr; i < itcntr+8; i++){
        ((uint8_t*)&vz2)[i-itcntr]= a[i];
    }

}


int AlgorithmService::sign_fn(double a)
{
    if (a == 0) return 0;
    if (a < 0) return -1;
    if (a > 0) return 1;
}

/*
 * Description: performs step3 of detumbling on ADCS - Sensor readings and fusion
 * input:  weights, raw and bias values of each sensor
 * output: normalized and fused magnetic field readings.
 */
void AlgorithmService::step3_biasCalc(double w1, vector_t b1_raw, vector_t b1_bias, double w2, vector_t b2_raw, vector_t b2_bias, vector_t* b_cur, vector_t* b_cur_norm)
{
    //vector_t b1_cur; b2_cur; b1_bias; b2_bias; b1_raw; b2_raw, b_cur;
    vector_t b1_cur, b2_cur;
    /*equation 1a*/
    b1_cur.x = T1[0][0]*b1_raw.x + T1[0][1]*b1_raw.y + T1[0][2]*b1_raw.z + b1_bias.x;
    b1_cur.y = T1[1][0]*b1_raw.x + T1[1][1]*b1_raw.y + T1[1][2]*b1_raw.z + b1_bias.y;
    b1_cur.z = T1[2][0]*b1_raw.x + T1[2][1]*b1_raw.y + T1[2][2]*b1_raw.z + b1_bias.z;

    /*equation 1b*/
    b2_cur.x = T2[0][0]*b2_raw.x + T2[0][1]*b2_raw.y + T2[0][2]*b2_raw.z + b2_bias.x;
    b2_cur.y = T2[1][0]*b2_raw.x + T2[1][1]*b2_raw.y + T2[1][2]*b2_raw.z + b2_bias.y;
    b2_cur.z = T2[2][0]*b2_raw.x + T2[2][1]*b2_raw.y + T2[2][2]*b2_raw.z + b2_bias.z;

    /*equation 2*/
    b_cur->x = w1*b1_cur.x + w2*b2_cur.x;
    b_cur->y = w1*b1_cur.y + w2*b2_cur.y;
    b_cur->z = w1*b1_cur.z + w2*b2_cur.z;

    double temp_norm = sqrt(b_cur->x*b_cur->x + b_cur->y*b_cur->y + b_cur->z*b_cur->z);
    b_cur_norm->x = b_cur->x / temp_norm;
    b_cur_norm->y = b_cur->y / temp_norm;
    b_cur_norm->z = b_cur->z / temp_norm;

}

/*
 * Description: performs step4 of the detumbling on ADCS - Bdot calculation
 * input: b_cur, b_prev, b_cur_norm, b_prev_norm
 * output: b_dot, b_dot_norm
 */
void AlgorithmService::step4_bdotCalc(vector_t b_cur, vector_t b_cur_norm, vector_t b_prev, vector_t b_prev_norm,
                    vector_t* b_dot, vector_t* b_dot_norm)
{
    static bool first_call = true;
    if (first_call == true) {
        b_dot->x      = 0;
        b_dot->y      = 0;
        b_dot->z      = 0;
        b_dot_norm->x = 0;
        b_dot_norm->y = 0;
        b_dot_norm->z = 0;
        first_call = false;
    }
    // make sure Tc is 250 ms
    else {
        b_dot->x = (b_cur.x - b_prev.x)/Tc;
        b_dot->y = (b_cur.y - b_prev.y)/Tc;
        b_dot->z = (b_cur.z - b_prev.z)/Tc;

        b_dot_norm->x = (b_cur_norm.x - b_prev_norm.x)/Tc;
        b_dot_norm->y = (b_cur_norm.y - b_prev_norm.y)/Tc;
        b_dot_norm->z = (b_cur_norm.z - b_prev_norm.z)/Tc;
    }
}
/*
 * Description: performs step5 of the detumbling algorithm - tumble parameter update
 * input: alpha, p_tumb, b_dot
 * output: tumble parameter
 */
vector_t AlgorithmService::step5_tumbleParam(vector_t b_dot_norm)
{
    static vector_t p_tumb = {2,2,2};
    static bool first_call = true;

    p_tumb.x = alpha* fabs(b_dot_norm.x) + (1-alpha)*p_tumb.x;
    p_tumb.y = alpha* fabs(b_dot_norm.y) + (1-alpha)*p_tumb.y;
    p_tumb.z = alpha* fabs(b_dot_norm.z) + (1-alpha)*p_tumb.z;

    if (first_call == true) {
        p_tumb.x = 2; p_tumb.y = 2; p_tumb.z = 2;
        first_call = false;
    }
    return p_tumb;
}

/*
 * Description: performs step6 of the detumbling algorithm - counter update
 * input: p_tumb
 * output: c_tumble
 * Make sure the counter is unsigned int, unclear overflow errors upon saturation
 */
void AlgorithmService::step6_countUpdate(vector_t p_tumb, unsigned int* c_tumb, unsigned int* c_detumb)
{

    if (p_tumb.x <= detumble_p_bar_low && p_tumb.y <= detumble_p_bar_low && p_tumb.z <= detumble_p_bar_low) {
        // saturate timer, avoid overflows
        if (*c_detumb < 65535) {
            *c_detumb = *c_detumb + 1;
        }
    }
    else {
        *c_detumb = 0;
    }

    if (p_tumb.x >= detumble_p_bar_upp || p_tumb.y >= detumble_p_bar_upp || p_tumb.z >= detumble_p_bar_upp) {
        // saturate timer, avoid overflows
        if (*c_tumb < 65535) {
            *c_tumb = *c_tumb + 1;
        }
    }
    else {
        *c_tumb = 0;
    }
}

/*
 * Description: performs step7 of the detumbling algorithm - decision to actuate
 * input: c_tumble, c_detumble
 * output: returns bool chi_tumb
 */
int AlgorithmService::step7_assessRotation(unsigned int* c_tumb, unsigned int* c_detumb) //, vector_t b_cur, vector_t b_dot_norm, vector_t* t_on, vector_t* s_on)
{
    // NB: Should be static!!
    static int chi_tumb = 0;
    if (*c_tumb * Tc >= t_conf_tumb) {
        chi_tumb = 1;
    }
    if (*c_detumb * Tc >= t_conf_detumb) {
        chi_tumb = 0;
    }
    return chi_tumb;
}

/*
 * Description: performs step9 of the detumbling algorithm - how much to actuate
 * input: k_w, b_cur, d_dot_norm
 * output: desired magnetic moment
 */
vector_t AlgorithmService::step9_controlCalc(vector_t b_cur, vector_t b_dot_norm)
{
    vector_t m_des;
    double temp_norm = sqrt(b_cur.x*b_cur.x + b_cur.y*b_cur.y + b_cur.z*b_cur.z);
    m_des.x = -k_w * b_dot_norm.x / temp_norm;
    m_des.y = -k_w * b_dot_norm.y / temp_norm;
    m_des.z = -k_w * b_dot_norm.z / temp_norm;
    return m_des;
}

/*
 * Description: performs step10 of the detumbling algorithm - actually actuate
 * input: desired magnetic moment, polarity
 * output: t_on
 */
void AlgorithmService::step10_torqueActuate(vector_t m_des, vector_t m_pol, vector_t* t_on, vector_t* s_on)
{
    Ta = delta*Tc;

    vector_t var;
    var.x = fabs(m_des.x);
    var.y = fabs(m_des.y);
    var.z = fabs(m_des.z);

    var.x = var.x / m_max.x;
    var.y = var.y / m_max.y;
    var.z = var.z / m_max.z;

    // magnitude
    t_on->x = Ta * MIN(1, var.x);
    t_on->y = Ta * MIN(1, var.y);
    t_on->z = Ta * MIN(1, var.z);

    // direction
    s_on->x = m_pol.x * sign_fn(m_des.x);
    s_on->y = m_pol.y * sign_fn(m_des.y);
    s_on->z = m_pol.z * sign_fn(m_des.z);
    // TODO: add actuation signals
}

/*
 * Description: Hold for desaturate/hysterisis
 * input: control, hold and actuation time
 * output: stop processor in the meantime
 */
double AlgorithmService::step11_hold()
{
    return 0;
}

// Use above functions in control loop
vector_t AlgorithmService::controlLoop(vector_t b1_raw, vector_t b2_raw,
                 vector_t* s_on, vector_t* t_on, vector_t* p_tumb,
                 unsigned int* c_tumb, unsigned int* c_detumb)
{
        vector_t b_cur, b_cur_norm, b_dot, b_dot_norm;
        static vector_t b_prev = { 0,0,0 };
        static vector_t b_prev_norm = { 0,0,0 };
        vector_t m_des = { 0,0,0 };
        vector_t m_pol = { 1,1,1 };

        // since old values are not used, might as well clear them to avoid garbage when actuation period is zero
        t_on->x = 0;
        t_on->y = 0;
        t_on->z = 0;
        s_on->x = 0;
        s_on->y = 0;
        s_on->z = 0;

        step3_biasCalc(0.5, b1_raw, b1_bias, 0.5, b2_raw, b2_bias, &b_cur, &b_cur_norm);

        step4_bdotCalc(b_cur, b_cur_norm, b_prev, b_prev_norm, &b_dot, &b_dot_norm);
        b_prev = b_cur;
        b_prev_norm = b_cur_norm;

        *p_tumb = step5_tumbleParam(b_dot_norm);
        step6_countUpdate(*p_tumb, c_tumb, c_detumb);

        if (step7_assessRotation(c_tumb, c_detumb) > 0) {
            m_des = step9_controlCalc(b_cur, b_dot_norm);
            step10_torqueActuate(m_des, m_pol, t_on, s_on);
            step11_hold();
        }
        else {
            step11_hold();
        }
        return m_des;
        // TODO: Step8, actuation
}
