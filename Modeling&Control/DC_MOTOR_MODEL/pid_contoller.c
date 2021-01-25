
#define KP 0	   
#define KI 0		
#define KD 0	
#define Sampling_Time 0.1 	
#define Max_out 12
#define Max_PWM 255


int Last_Error = 0 ; // Global Variable
int Volt_Out = 0 ; 


int PID (int Req_Speed , int Motor_Speed )
{
    int Error = 0 ;
    int P_Error = 0 ; 
    int I_Error = 0 ; 
    int D_Error = 0 ; 
    int PWM = 0 ;
    
    Error = Req_Speed - Motor_Speed ;

    P_Error = KP * Error;
    I_Error = KI * (Error + Last_Error) * Sampling_Time ;
    D_Error = (KD * (Error - Last_Error)) / Sampling_Time ;

    Volt_Out = Volt_Out + P_Error + I_Error + D_Error ;

    Last_Error = Error;

    if (Volt_Out >= 0 )
    {
        Set_Motor_Diriction(CCW);
    }
    else
    {
        Set_Motor_Diriction(CW);
       Volt_Out = Volt_Out * -1 ;
    }
    if (Volt_Out >= Max_out)
    {
        Volt_Out = Max_out ; 
    }
    
    PWM = (Volt_Out/Max_out)*Max_PWM

    return PWM ;
}


/*
    FilterCoefficient = (Kd * Error - Filter_DSTATE) * N;
    Volt_Out = (Kp * Error + Integrator_DSTATE) + FilterCoefficient;
    Integrator_DSTATE += Ki * (Error + Last_Error) * 0.01;
    Filter_DSTATE += 0.01 * FilterCoefficient;
	*/