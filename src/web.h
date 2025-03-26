

#ifndef _WEB_H
#define _WEB_H

#include "main.h"
#include "riot.h"
#include "motion.h"
#include "textfile.h"
#include "routines.h"

#define HTTP_SERVER_PORT      80
#define OTA_SERVER_PORT      8080

// Configuration webserver
bool startBonjour(void);
void startWebServer(void);
void handleNotFound(void);
void handleParams(void);
void handleFillForm(void);
void displayArgs(int ArgsNumber);

// OTA
void onOTAStart();
void onOTAProgress(size_t current, size_t final);
void onOTAEnd(bool success);


#endif
          
                             
                             
                             
                             
                             
