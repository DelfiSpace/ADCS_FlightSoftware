/*
 * AlgorithmService.h
 *
 *  Created on: 17 Jun 2020
 *      Author: Casper, Bart
 */

#ifndef ALGORITHMSERVICE_H_
#define ALGORITHMSERVICE_H_

#include "Service.h"
#include "Console.h"
#include <math.h>

#define ALGORITHM_SERVICE 25

// math macros
#define SGN(a)   (((a)<(0))?(-1):(1))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

typedef struct {
    double x;
    double y;
    double z;
} vector_t;

class AlgorithmService : public Service
{
private:
    uint8_t calcBuf[100] = {0}; //buffer for output

    //DETUMBLE ALGO VARIABLES

    signed char T1[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, -1};
    signed char T2[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, -1};
    vector_t b1_bias = {0,0,0};
    vector_t b2_bias = {0,0,0};

    double Tc = 0.25; // 4 Hz, 250 ms
    double alpha = 0.01;

    double detumble_p_bar_upp = 0.085;
    double detumble_p_bar_low = 0.075;

    double t_conf_tumb = 10; //120;
    double t_conf_detumb = 3600;

    double k_w = 0.0000012073917;

    double delta = 0.6;
    double Ta;

    vector_t m_max = {0.002, 0.002, 0.002};
    vector_t m_pol = { 1, 1, 1 };

    double Ts = 0.02;

    // initialize detumble variables
    vector_t mag1Data = {0,0,0};
    vector_t mag2Data = {0,0,0};
    vector_t s_on     = {0,0,0};
    vector_t t_on     = {0,0,0};
    vector_t p_tumb   = {0,0,0};

    // make sure to leave counts below static, they aren't remembered by controlLoop,
    // they are just incremented by controlLoop
    static unsigned int c_tumb;
    static unsigned int c_detumb;

public:
     virtual bool process( DataMessage &command, DataMessage &workingBbuffer );

     // DETUMBLE ALGO FUNCTIONS
     void input2doubles(uint8_t a[], double& vx1, double& vy1, double& vz1, double& vx2, double& vy2, double& vz2);
     int sign_fn(double a);
     void step3_biasCalc(double w1, vector_t b1_raw, vector_t b1_bias, double w2, vector_t b2_raw, vector_t b2_bias, vector_t* b_cur, vector_t* b_cur_norm);
     void step4_bdotCalc(vector_t b_cur, vector_t b_cur_norm, vector_t b_prev, vector_t b_prev_norm, vector_t* b_dot, vector_t* b_dot_norm);
     vector_t step5_tumbleParam(vector_t b_dot);
     void step6_countUpdate(vector_t p_tumb, unsigned int* c_tumb, unsigned int* c_detumb);
     int step7_assessRotation(unsigned int* c_tumb, unsigned int* c_detumb);//, vector_t b_cur, vector_t b_dot_norm, vector_t* t_on, vector_t* s_on);
     // TODO: Step8 to be implemented on OBC
     vector_t step9_controlCalc(vector_t b_cur, vector_t b_dot_norm);
     void step10_torqueActuate(vector_t m_des, vector_t m_pol, vector_t* t_on, vector_t* s_on);
     double step11_hold();

     vector_t controlLoop(vector_t b1_raw, vector_t b2_raw,
                      vector_t* s_on, vector_t* t_on, vector_t* p_tumb,
                      unsigned int* c_tumb, unsigned int* c_detumb);
};



#endif /* ALGORITHMSERVICE_H_ */
