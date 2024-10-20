#include "fancontrol.h"
#include "tmp451.h"

//Fan curve table
const TemperaturePoint defaultTable[] =
{
    { .temperature_c = 20,  .fanLevel_f = 0.1   },
    { .temperature_c = 40,  .fanLevel_f = 0.5   },
    { .temperature_c = 50,  .fanLevel_f = 0.6   },
    { .temperature_c = 60,  .fanLevel_f = 0.7   },
    { .temperature_c = 100, .fanLevel_f = 1     }
};

TemperaturePoint *fanControllerTable;

//Fan
Thread FanControllerThread;
bool fanControllerThreadExit = false;

//Log
char logPath[PATH_MAX];

void CreateDir(char *dir)
{
    char dirPath[PATH_MAX];

    for(int i = 0; i < PATH_MAX; i++)
    {
        if(*(dir + i) == '/' && access(dirPath, F_OK) == -1)
        {
            mkdir(dirPath, 0777);
        }
        dirPath[i] = *(dir + i);
    }
}

void InitLog()
{
    if(access(LOG_DIR, F_OK) == -1)
        CreateDir(LOG_DIR);

    if(access(LOG_FILE, F_OK) != -1)
        remove(LOG_FILE);
}

void WriteLog(char *buffer)
{
    FILE *log = fopen(LOG_FILE, "a");
    if(log != NULL)
    {
        fprintf(log, "%s\n", buffer);  
    }
    fclose(log);
}

void WriteConfigFile(TemperaturePoint *table)
{
    if(table == NULL)
    {
        table = malloc(sizeof(defaultTable));
        memcpy(table, defaultTable, sizeof(defaultTable));
    }

    if(access(CONFIG_DIR, F_OK) == -1)
        CreateDir(CONFIG_DIR);

    FILE *config = fopen(CONFIG_FILE, "w");
    fwrite(table, TABLE_SIZE, 1, config);
    fclose(config);
}



void ReadConfigFile(TemperaturePoint **table_out)
{
    InitLog();

    *table_out = malloc(sizeof(defaultTable));
    memcpy(*table_out, defaultTable, sizeof(defaultTable));

    if(access(CONFIG_DIR, F_OK) == -1)
    {
        CreateDir(CONFIG_DIR);
        WriteConfigFile(NULL);
        
        WriteLog("Missing config dir");
    }
    else
    {
        if(access(CONFIG_FILE, F_OK) == -1)
        {
            WriteConfigFile(NULL);
            
            WriteLog("Missing config file");
        }
        else
        {
            FILE *config = fopen(CONFIG_FILE, "r");
            fread(*table_out, TABLE_SIZE, 1, config);
            fclose(config);

            WriteLog("config file exist");
        }
    }    
}

void InitFanController(TemperaturePoint *table)
{
    fanControllerTable = table;

    if(R_FAILED(threadCreate(&FanControllerThread, FanControllerThreadFunction, NULL, NULL, 0x4000, 0x3F, -2)))
    {
        WriteLog("Error creating FanControllerThread");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }
}

void FanControllerThreadFunction(void*)
{
    FanController fc;
    float fanLevelSet_f = 0;
    float temperatureC_f = 0;

    Result rs = fanOpenController(&fc, 0x3D000001);
    if(R_FAILED(rs))
    {
        WriteLog("Error opening fanController");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }

    while(!fanControllerThreadExit)
    {
        rs = Tmp451GetSocTemp(&temperatureC_f);
        if(R_FAILED(rs))
        {
            WriteLog("tsSessionGetTemperature error");
            diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
        }

        if(temperatureC_f >= 0 && temperatureC_f <= fanControllerTable->temperature_c)
        {
            float m = 0;
            float q = 0;

            m = fanControllerTable->fanLevel_f / fanControllerTable->temperature_c;
            q = 0 - m;

            fanLevelSet_f = (m * temperatureC_f) + q;

        }else if(temperatureC_f >= (fanControllerTable + 4)->temperature_c)
        {
            fanLevelSet_f = (fanControllerTable + 4)->fanLevel_f;
        }else
        {
            for(int i = 0; i < (TABLE_SIZE/sizeof(TemperaturePoint)) - 1; i++)
            {
                if(temperatureC_f >= (fanControllerTable + i)->temperature_c && temperatureC_f <= (fanControllerTable + i + 1)->temperature_c)
                {
                    float m = 0;
                    float q = 0;

                    m = ((fanControllerTable + i + 1)->fanLevel_f - (fanControllerTable + i)->fanLevel_f ) / ((fanControllerTable + i + 1)->temperature_c - (fanControllerTable + i)->temperature_c);
                    q = (fanControllerTable + i)->fanLevel_f - (m * (fanControllerTable + i)->temperature_c);

                    fanLevelSet_f = (m * temperatureC_f) + q;
                    break;
                }
            }
        }

        rs = fanControllerSetRotationSpeedLevel(&fc, fanLevelSet_f);
        if(R_FAILED(rs))
        {
            WriteLog("fanControllerSetRotationSpeedLevel error");
            diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
        }
        svcSleepThread(100000000); //100'000'000
    }

    fanControllerClose(&fc);
}

void StartFanControllerThread()
{
    if(R_FAILED(threadStart(&FanControllerThread)))
    {
        WriteLog("Error starting FanControllerThread");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }
}

void CloseFanControllerThread()
{   
    Result rs;
    fanControllerThreadExit = true;
    rs = threadWaitForExit(&FanControllerThread);
    if(R_FAILED(rs))
    {
        WriteLog("Error waiting fanControllerThread");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }
    threadClose(&FanControllerThread);
    if(R_FAILED(rs))
    {
        WriteLog("Error closing fanControllerThread");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }
    fanControllerThreadExit = false;
    free(fanControllerTable);
}

void WaitFanController()
{
    if(R_FAILED(threadWaitForExit(&FanControllerThread)))
    {
        WriteLog("Error waiting fanControllerThread");
        diagAbortWithResult(MAKERESULT(Module_Libnx, LibnxError_ShouldNotHappen));
    }
}
