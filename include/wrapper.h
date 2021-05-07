#ifdef __cplusplus
extern "C" {
#endif
void RunPandarSwiftSDK(char* deviceipaddr, int lidarport, int gpsport, char* correctionfile, char* firtimeflie, char* pcapfile, int viewMode,
                            char* certFile, char* privateKeyFile, char* caFile, int runTime);                           
void StopPandarSwiftSDK();
void SetPcdFileWriteFlag(int flag ,int frameNum);
void SetPrintFlag(int flag);

#ifdef __cplusplus
};
#endif