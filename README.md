# Fault_Diagnosis_BrushlessDCMotor
These programs were made in order to check how the fault diagnosis algorithm works with data from the DCmotor equivalent to BLDC simulator. For Bachelor's Thesis on February-June 2020.  
If you want to performance a fault escenario you need to set the parameters of your motor and run both Simulador_MotorDC_Equivalent_BLDC.m and Simulador_MotorDC_Equivalent_BLDC.slx. As a result, graphics about residuals evalution and data file are generated. The latter will be loaded on the algorithm code to perform fault diagnosis. 
To set the fault diagnosis algorithm or Algorisme_FDI.m you need a data file with four residuals data, four thresholds data, current signal and speed signal. The algorihm requires correlation_funciton.m to compute both correlation values.  
