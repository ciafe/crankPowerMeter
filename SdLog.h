
#ifndef SDLOG_API
#define SDLOG_API

boolean Logging_Init_SD(void);

boolean Logging_CreateNewLoggingFile(void);

void    Logging_StoreMeasurementData(void);

void    Logging_CloseSDFile(void);


#endif
