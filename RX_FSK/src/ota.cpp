#include <arduino.h>
#include <ESPAsyncWebServer.h>
#include "ota.h"
#include "git_version.h"
#include "common.h"
#include "Sonde.h"
#include "Display.h"
//#include <WiFi.h>
#include <Update.h>

#define OTA_FIRMWARE   0
#define OTA_FILESYSTEM 1

//const char *updateHost = "rdzsonde.tuu.fi";
const char *updateHost = "heittoka.kapsi.fi";
int updatePort = 80;

const char *updatePrefixM = "/master/";
//const char *updatePrefixD = "/multi_ch/devel/";
const char *updatePrefixD = "/devel/";
const char *updatePrefix = updatePrefixM;

uint8_t ota_image_type;

extern HardwareSerial Serial;
extern WiFiClient client;

int fetchHTTPheader(int *validType);

const char *ota_create_form(boolean run, char *message) {
#if 1
  char *ptr = message;
  strcpy(ptr, "<html><head><link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\"></head><body><form action=\"ota.html\" method=\"post\">");
  if (run) {
    strcat(ptr, "<p>Doing update, wait until reboot</p>");
  } else {
    sprintf(ptr + strlen(ptr), "<p>Currently installed: %s-%c%d</p>\n", version_id, SPIFFS_MAJOR + 'A' - 1, SPIFFS_MINOR);
    strcat(ptr, "<p>Available release:: <iframe src=\"http://rdzsonde.mooo.com/master/update-info.html\" style=\"height:40px;width:400px\"></iframe><br>"
           "Available devel: <iframe src=\"http://rdzsonde.mooo.com/devel/update-info.html\" style=\"height:40px;width:400px\"></iframe></p>");
    strcat(ptr, "<input type=\"submit\" name=\"master\" value=\"Master-Update\"></input><br><input type=\"submit\" name=\"devel\" value=\"Devel-Update\">");
    strcat(ptr, "<br><p>Note: If suffix is the same, update should work fully. If the number is different, update contains changes in the file system. A full re-flash is required to get all new features, but the update should not break anything. If the letter is different, a full re-flash is mandatory, update will not work</p>");
  }
  strcat(ptr, "</form></body></html>");
  //Serial.printf("Update form: size=%d bytes\n", strlen(message));
#endif // 0
  return message;
}

const char *ota_handle_post(AsyncWebServerRequest * request) {
#if 1
  Serial.println("Handling post request");
  int params = request->params();
  for (int i = 0; i < params; i++) {
    String param = request->getParam(i)->name();
    Serial.println(param.c_str());
    if (param.equals("devel")) {
      Serial.println("equals devel");
      updatePrefix = updatePrefixD;
    }
    else if (param.equals("master")) {
      Serial.println("equals master");
      updatePrefix = updatePrefixM;
    }
    else if (param.equals("firmware")) {
      Serial.println("equals firmware");
      ota_image_type = OTA_FIRMWARE;
    }
    else if (param.equals("filesystem")) {
      Serial.println("equals filesystem");
      ota_image_type = OTA_FILESYSTEM;
    }
  }
  Serial.printf("Updating: %supdate.ino.bin\n", updatePrefix);
  enterMode(3 /*ST_UPDATE*/); // tää käynnistää execOTA()
#endif // 0
  return "";
}

#if 1
/// Testing OTA Updates
/// somewhat based on Arduino's AWS_S3_OTA_Update
// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

// OTA Logic
void execOTA() {
  int contentLength = 0;
  bool isValidContentType = false;
  sonde.clearDisplay();
  uint8_t dispxs, dispys;
  if ( ISOLED(sonde.config) ) {
    disp.rdis->setFont(FONT_SMALL);
    dispxs = dispys = 1;
    char uh[17];
    strncpy(uh, updateHost, 17);
    uh[16] = 0;
    disp.rdis->drawString(0, 0, uh);
  } else {
    disp.rdis->setFont(5);
    dispxs = 18;
    dispys = 20;
    disp.rdis->drawString(0, 0, updateHost);
  }

  if (ota_image_type == OTA_FILESYSTEM) {
#if 1
  //Serial.print("Connecting to: "); Serial.println(updateHost);
  // Connect to Update host
  if (!client.connect(updateHost, updatePort)) {
    //Serial.println("Connection to " + String(updateHost) + " failed. Please check your setup");
    return;
  }

  // First, update file system
  //Serial.println("Fetching fs update");
  disp.rdis->drawString(0, 1 * dispys, "Fetching fs...");
  client.printf("GET %supdate.fs.bin HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n", updatePrefix, updateHost);
  // see if we get some data....

  int type = 0;
  int res = fetchHTTPheader(&type);
  if (res < 0) {
    // Back to some normal state
    enterMode(0 /*ST_DECODER*/);
    return;
  }
  // process data...
  while (client.available()) {
    // get header...
    char fn[128];
    fn[0] = '/';
    client.readBytesUntil('\n', fn + 1, 128);
    char *sz = strchr(fn, ' ');
    if (!sz) {
      client.stop();
      return;
    }
    *sz = 0;
    int len = atoi(sz + 1);
    //Serial.printf("Updating file %s (%d bytes)\n", fn, len);
    char fnstr[17];
    memset(fnstr, ' ', 16);
    strncpy(fnstr, fn, 16);
    fnstr[16] = 0;
    disp.rdis->drawString(0, 2 * dispys, fnstr);
    File f = SPIFFS.open(fn, FILE_WRITE);
    // read sz bytes........
    while (len > 0) {
      unsigned char buf[1024];
      int r = client.read(buf, len > 1024 ? 1024 : len);
      if (r == -1) {
        client.stop();
        return;
      }
      f.write(buf, r);
      len -= r;
    }
  }
  client.stop();
#endif // 0
  }

  if (ota_image_type == OTA_FIRMWARE) {
  Serial.print("Connecting to: "); Serial.println(updateHost);
  // Connect to Update host
  if (!client.connect(updateHost, updatePort)) {
    Serial.println("Connection to " + String(updateHost) + " failed. Please check your setup");
    return;
  }

  // Connection succeeded, fecthing the bin
  //Serial.printf("Fetching bin: %supdate.ino.bin\n", updatePrefix);
  disp.rdis->drawString(0, 3 * dispys, "Fetching update");
  Serial.println("A");

  // Get the contents of the bin file
  client.printf("GET %supdate.ino.bin HTTP/1.1\r\n"
                "Host: %s\r\n"
                "Cache-Control: no-cache\r\n"
                "Connection: close\r\n\r\n",
                updatePrefix, updateHost);
  Serial.println("AA");
  // Check what is being sent
  //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
  //                 "Host: " + host + "\r\n" +
  //                 "Cache-Control: no-cache\r\n" +
  //                 "Connection: close\r\n\r\n");

  int validType = 0;
  contentLength = fetchHTTPheader( &validType );
  Serial.println("AAA");
  if (validType == 1) isValidContentType = true;

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));
  disp.rdis->drawString(0, 4 * dispys, "Len: ");
  String cls = String(contentLength);
  disp.rdis->drawString(5 * dispxs, 4 * dispys, cls.c_str());

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      disp.rdis->drawString(0, 5 * dispys, "Starting update");
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        //Serial.println("Written : " + String(written) + " successfully");
      } else {
        //Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        //Serial.println("OTA done!");
        if (Update.isFinished()) {
          //Serial.println("Update successfully completed. Rebooting.");
          disp.rdis->drawString(0, 7 * dispys, "Rebooting....");
          delay(1000);
          ESP.restart();
        } else {
          //Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        //Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      //Serial.println("Not enough space to begin OTA");
      client.flush();
    }
  } else {
    //Serial.println("There was no content in the response");
    client.flush();
  }
  }
  // Back to some normal state
  enterMode(0 /*ST_DECODER*/);
  Serial.println("AAAA");
}

int fetchHTTPheader(int *validType) {
  int contentLength = -1;
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println("Client Timeout !");
      client.stop();
      return -1;
    }
  }
  // Once the response is available, check stuff

  /*
     Response Structure
      HTTP/1.1 200 OK
      x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
      x-amz-request-id: 2D56B47560B764EC
      Date: Wed, 14 Jun 2017 03:33:59 GMT
      Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
      ETag: "d2afebbaaebc38cd669ce36727152af9"
      Accept-Ranges: bytes
      Content-Type: application/octet-stream
      Content-Length: 357280
      Server: AmazonS3

      {{BIN FILE CONTENTS}}

  */
  while (client.available()) {
    Serial.println("While()");
    // read line till \n
    String line = client.readStringUntil('\n');
    // remove space, to check if the line is end of headers
    line.trim();

    // if the the line is empty,
    // this is end of headers
    // break the while and feed the
    // remaining `client` to the
    // Update.writeStream();
    if (!line.length()) {
      //headers ended
      break; // and get the OTA started
    }

    // Check if the HTTP Response is 200
    // else break and Exit Update
    if (line.startsWith("HTTP/1.1")) {
      if (line.indexOf("200") < 0) {
        Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
        return -1;
      }
    }

    // extract headers here
    // Start with content length
    if (line.startsWith("Content-Length: ")) {
      contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
      Serial.println("Got " + String(contentLength) + " bytes from server");
    }

    // Next, the content type
    if (line.startsWith("Content-Type: ")) {
      String contentType = getHeaderValue(line, "Content-Type: ");
      Serial.println("Got " + contentType + " payload.");
      if (contentType == "application/octet-stream") {
        if (validType) *validType = 1;
      }
    }
  }
  Serial.println("END - END");
  Serial.println(contentLength);
  return contentLength;
}
#endif // 0