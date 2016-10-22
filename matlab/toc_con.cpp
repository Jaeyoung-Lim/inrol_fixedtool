#define S_FUNCTION_NAME toc_con /* Defines and Includes */
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

//------------------------------------------------------------------
#include "Eigen/core"
using namespace Eigen;
using namespace std;
//------------------------------------------------------------------





static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }
    
    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 52); //number of inputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    
    if (!ssSetNumOutputPorts(S,1)) return;
    ssSetOutputPortWidth(S, 0, 4); //number of outputs
    
    ssSetNumSampleTimes(S, 1);
    
    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

// the main control code here
static Vector4d toc_controller(VectorXd input_u){//, VectorXd feedback_in, VectorXd parameter_in){
    // based on H.-N. Nguyen et al. / Automatica 61 (2015) 289-301
   
    //--------------------------------------------------
    // input
    int ch_index = 0;

    //force feedback channel
    Vector3d f_e(input_u(0),input_u(1), input_u(2));
    Vector3d if_e(input_u(3), input_u(4), input_u(5));

    //desired motion channel
    ch_index = 5;
    Vector3d y_d(input_u(ch_index + 1),input_u(ch_index + 2),input_u(ch_index + 3)); // desired motion of the tool tip y
    Vector3d y_d_dot(input_u(ch_index + 4), input_u(ch_index + 5), input_u(ch_index + 6));
    Vector3d y_d_ddot(input_u(ch_index + 7), input_u(ch_index + 8), input_u(ch_index + 9));
    Vector3d y_d_ddd(input_u(ch_index + 10), input_u(ch_index + 11), input_u(ch_index + 12));

    

    //desired force 
    ch_index = 5 + 12;
    Vector3d f_d(input_u(ch_index + 1));
    Vector3d if_d(input_u(ch_index + 2));

    //Motion feedback information
    ch_index = 5 + 12 + 2 ;
    Vector3d y_dot(input_u(ch_index + 1), input_u(ch_index + 2), input_u(ch_index + 3)); // translation velocity of the tool tip y 
    Vector3d y(input_u(ch_index + 4), input_u(ch_index + 5), input_u(ch_index + 6));
    Vector3d w(input_u(ch_index + 7), input_u(ch_index + 8), input_u(ch_index + 9)); // angular velocity of the quadrotor
    
    ch_index = 5 + 12 + 2 + 9;
    Matrix3d R; //rotation matrix of the quadrotor wrt the fixed frame
    R(0,0) = input_u(ch_index + 1);
    R(0,1) = input_u(ch_index + 2);
    R(0,2) = input_u(ch_index + 3); 
    R(1,0) = input_u(ch_index + 4);
    R(1,1) = input_u(ch_index + 5);
    R(1,2) = input_u(ch_index + 6); 
    R(2,0) = input_u(ch_index + 7);
    R(2,1) = input_u(ch_index + 8);
    R(2,2) = input_u(ch_index + 9);

    ch_index = 37; //5 + 12 + 2 + 9+ 9;
    Vector3d y_ddot(input_u(ch_index + 1),input_u(ch_index + 2),input_u(ch_index + 3)); 
    // acceleration of the tool tip - in case you want to feed forward this in the force control. In this file, just ignore this



    //--------------------------------------------------
    // system parameters
    ch_index = 40;
    double m = input_u(ch_index + 1); // masss
    Matrix3d J; //inertia
     J(0,0) = input_u(ch_index + 2);
     J(1,1) = input_u(ch_index + 3);
     J(2,2) = input_u(ch_index + 4);

    Vector3d d(input_u(ch_index + 5),input_u(ch_index + 6),input_u(ch_index + 7)); // tool tip position in the body frame

    //--------------------------------------------------
    // control gains
     ch_index = 47;
    double b =input_u(ch_index + 1); // the gains
    double k =input_u(ch_index + 2); 
    double k_w = input_u(ch_index + 3); // gain for controlling omega to omega_d
    double k_w_d = input_u(ch_index + 4); // gain for controlling omega to omega_d



    // some other constants
    double stp = 0.001; //time step
    double g = 9.81; //gravity



    //--------------------------------------------------
    //calculation

    Matrix3d S; //skew matrix S(omega)
    S(0,0) = 0;
    S(0,1) = -w(2);
    S(0,2) = w(1);
    S(1,0) = w(2);
    S(1,1) = 0;
    S(1,2) = -w(0);
    S(2,0) = -w(1);
    S(2,1) = w(0);
    S(2,2) = 0.0;
    
    Matrix3d Sd; //skew matrix S(d)
    Sd(0,0) = 0.0;
    Sd(0,1) = -d(2);
    Sd(0,2) = d(1);
    Sd(1,0) = d(2);
    Sd(1,1) = 0.0;
    Sd(1,2) = -d(0);
    Sd(2,0) = -d(1);
    Sd(2,1) = d(0);
    Sd(2,2) = 0.0;

    Vector3d Rfe = R.transpose()*f_e; 
    Vector3d tau_e = Sd*Rfe;//external torque on the quadrotor - \tau_c eq. (2) 
    
    // motion control  eq. (4) with this desired control u
    
   
    Vector3d u_c = y_d_ddot*m - (y_dot - y_d_dot)*b - (y - y_d)*k; 

    


    // the gain in lower force control layer
  /*  double k_d = 30; 
    double k_f =5; 
    double k_i = 6; 
    double m_a = 0.48;
    double eps = 0.5;

    double k_v = 0.5;
    double k_p = 0.3;
    */


    double k_star =-1.1*d(0)/d(2); // gain for nu_3 action -  gamma in eq. (16)
    double u_bar = 2; // gain for nu_3 action -  eto in eq. (16)


    double alpha_o = sqrt(d(0)*d(0)+d(2)*d(2)); // it is \bar{d} in (8)
    Matrix3d delta_o; //delta in (8)

    delta_o(0,0) =-d(2)/alpha_o;
    delta_o(0,1) = 0;
    delta_o(0,2) = d(0)/alpha_o;
    delta_o(1,0) = 0;
    delta_o(1,1) = 1;
    delta_o(1,2) = 0;
    delta_o(2,0) = d(0)/alpha_o;
    delta_o(2,1) = 0;
    delta_o(2,2) = d(2)/alpha_o;


    Vector3d nu_l = delta_o*w; //angular velocity transformation - eq. (8)
    Vector3d e3(0,0,1); // unit vector Oz

    Vector3d u_hat = R.transpose()*(u_c/m*(-1.0) + e3*9.81); //RHS of (9)
        


     
    

    Vector3d nu_d; 
    Vector3d nu_d_dot; //calculate \dot{nu} from (15) and (16)

    nu_d_dot(0) = -u_hat(1)/alpha_o - nu_l(1)*nu_l(2) ; 
    nu_d_dot(1) = -u_hat(0)/d(2)+(nu_l(0)*nu_l(0)+nu_l(1)*nu_l(1))*d(0)/d(2) + nu_l(0)*nu_l(2);    
    nu_d_dot(2) = k_star*(nu_d_dot(0)+nu_d_dot(0)*nu_l(1)*nu_l(1)+2*nu_l(0)*nu_l(1)*nu_d_dot(1)-atan(nu_l(0))*nu_d_dot(1)-nu_l(1)/(1+nu_l(0)*nu_l(0)));
    nu_d = nu_l+nu_d_dot*stp; // integrate nu_d
    nu_d(2) = k_star*nu_l(0)*(1+nu_l(1)*nu_l(1))-atan(nu_l(0))*k_star*nu_l(1)+atan(nu_l(0))*u_bar; //(16)

    //transform nu back to omega
    Vector3d w_d;
    Vector3d w_d_dot;
    w_d = delta_o*nu_d;
    w_d_dot = delta_o*nu_d_dot;


    // calculate for lambda (9) and tau (7)  
    double lambda = (u_hat(2) - d(0)*nu_d_dot(1)-d(2)*(nu_l(0)*nu_l(0)+nu_l(1)*nu_l(1))+d(0)*nu_l(0)*nu_l(2))*m;
    //using std::max;
    //using std::min;
    lambda = lambda<15?lambda:15;
    lambda = lambda>0?lambda:0;
    Vector3d tau;
    tau = S*J*w +J*(w_d_dot - (w-w_d)*k_w_d) - w*k_w - tau_e;
    
    
    printf("%f %f %f %f \n",lambda, tau(0),tau(1),tau(2));

   
   Vector4d control_ou(lambda,tau(0),tau(1),tau(2)); // put in the output of the function
   return control_ou;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T i;
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T *y = ssGetOutputPortRealSignal(S,0);
    int_T width = ssGetOutputPortWidth(S,0);

    int_T in_width = ssGetInputPortWidth(S,0);

    VectorXd input_u(in_width);

    for (i=0; i< in_width; i++){
        input_u(i) = *uPtrs[i];
   //     printf("%f\t", input_u(i));
    }
 
    

    
    Vector4d output_y = toc_controller(input_u);//, kk, kk);
    
    printf("\n");
    
    y[0] =  output_y(0);
    y[1] =  output_y(1);
    y[2] =  output_y(2);
    y[3] =  output_y(3);

    //printf("%f %f %f \n",y[1],y[2],y[3]);
    
}



static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
