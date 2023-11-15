#ifndef _OTA_H
#define _OTA_H

const char *ota_create_form(boolean run, char *message);
const char *ota_handle_post(AsyncWebServerRequest * request);
void execOTA();

#endif // _OTA_H