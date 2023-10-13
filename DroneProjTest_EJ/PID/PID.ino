float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0, 0, 0};

//PID Value (Retrieved from the video)
float PRateRoll=0.6 ; float PRatePitch=PRateRoll; float PRateYaw=2;
float IRateRoll=3.5 ; float IRatePitch=IRateRoll; float IRateYaw=12;
float DRateRoll=0.03 ; float DRatePitch=DRateRoll; float DRateYaw=0;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll=PIDReturn[0];
  PrevErrorRateRoll=PIDReturn[1]; 
  PrevItermRateRoll=PIDReturn[2];
}

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error; //P Equation
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2; //I Equation
  if (Iterm > 400){ // if statement to avoid Integral overshoot
    Iterm=400;
  }
  else if (Iterm <-400){
    Iterm=-400;
  }
  float Dterm=D*(Error-PrevError)/0.004; //D Equation
  float PIDOutput= Pterm+Iterm+Dterm; //PID Equation
  if (PIDOutput>400){ // if statement to avoid Integral overshoot
    PIDOutput=400;
  }
  else if (PIDOutput <-400){
    PIDOutput=-400;
  }
  
  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}