# Fault_Diagnosis_BrushlessDCMotor
This is a simulator for Fault Diagnosis of Brushless DC Motors based on Model-Approach with MATLAB-Simulink, which was elaborated during the completion of my Bachelor's Thesis at UPC. https://upcommons.upc.edu/handle/2117/330222   

If you want to performance a fault escenario you need to set the parameters of your motor and run both Simulador_MotorDC_Equivalent_BLDC.m and Simulador_MotorDC_Equivalent_BLDC.slx. As a result, graphics about residuals evalution and data file are generated. The latter will be loaded on the algorithm code to perform fault diagnosis. 

To set the fault diagnosis algorithm or Algorisme_FDI.m you need a data file with four residuals data, four thresholds data, current signal and speed signal. The algorithm requires correlation_funciton.m to compute both correlation values.  

The last update this simulator is dated June 2020, with the version of MATLAB 2020a. 
