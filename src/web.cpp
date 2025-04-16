
#include "web.h"

// Hooks for OTA
unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}


bool startBonjour(void) {
  bool mdnsOK = false;
  // mDNS - riot.local => uses the 'mdns' param (name) in config.txt
  // as an alternative, riot-xx:yy.local could be setup using the last hex numbers of the MAC address
  //uint64_t chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
  //uint8_t *id = (uint8_t*)&chipid;
  //char str[MAX_PATH_LEN];
  //sprintf(str, "riot-%02x:%02x\0", id[4], id[5]); // Uses the 6 HEX numbers of the MAC address in the AP/SSID name
  if (MDNS.begin(riot.getBonjour())) {
    mdnsOK = true;
  }
  else {
    Serial.println("Error setting up MDNS responder!");
  }
  if(mdnsOK) {
  // Add service to MDNS-SD
    MDNS.addService("_http", "_tcp", 80);
    //MDNS.addService("_riotSend", "_udp", riot.getDestPort());
    //MDNS.addService("_riotReceive", "_udp", riot.getReceivePort());
    Serial.printf("MDNS responder start on %s.local\n", riot.getBonjour());
  }
  return(mdnsOK);
}


void startWebServer(void) {
  Serial.printf("Starting webserver on port %d\n", HTTP_SERVER_PORT);
  // Setup All the callbacks
  httpServer.on ( "/params", handleParams );
  httpServer.onNotFound ( handleNotFound );
  httpServer.serveStatic("/", FFat, "/index.html");
  //httpServer.on("/", HTTP_GET, handleIndex);
  httpServer.serveStatic("/styles.css", FFat, "/styles.css");
  httpServer.serveStatic("/ircam.png", FFat, "/ircam.png");
  httpServer.serveStatic("/update", FFat, "/ota.html");
  httpServer.on("/getparams", HTTP_GET, handleFillForm);
  httpServer.begin();

  IPAddress tempIP;
  if (riot.getOperatingMode())
    tempIP = WiFi.softAPIP();
  else
    tempIP = riot.getOwnIP();

  Serial.printf("Configuration Webserver started on port %d - Open http://", HTTP_SERVER_PORT);
  Serial.println(tempIP);
  Serial.printf(":%d/ in your browser\n", HTTP_SERVER_PORT);

  // Start OTA
  ElegantOTA.begin(&httpServer);    // Start ElegantOTA - Can use arguments username & password to protect update.
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  Serial.printf("OTA Update Webserver started on port %d - Open http://", HTTP_SERVER_PORT);
  Serial.println(tempIP);
  Serial.printf(":%d/update in your browser\n", HTTP_SERVER_PORT);
}

void handleNotFound(void) {
  setLedColor(Purple);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += httpServer.uri();
  message += "\nMethod: ";
  message += ( httpServer.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += httpServer.args();
  message += "\n";

  for ( int i = 0; i < httpServer.args(); i++ ) {
    message += " " + httpServer.argName ( i ) + ": " + httpServer.arg ( i ) + "\n";
  }
  message += "\n";
  message += "*WAVES HAND* This is not the URL you are looking for\n";

  httpServer.send ( 404, "text/plain", message );
  setLedColor(Black);
}

// Send params to the webpage via JSON exchange
void handleFillForm(void) {
  IPAddress tempIP;
  String json = "{";
  int i;

  //Serial.printf("getparams web request\n");
  //Serial.println(String(WiFi.softAPmacAddress()));

  json += "\"dhcp\":" + String(riot.isDHCP()) + ",";
  json += "\"ssid\":\"" + String(riot.getSSID()) + "\",";
  json += "\"password\":\"" + String(riot.getPassword()) + "\",";

  json += "\"mac\":\"" + String(WiFi.macAddress()) + "\",";
  if(!riot.isForcedConfig() || riot.isConfig()) {
    json += "\"mac\":\"" + String(WiFi.softAPmacAddress()) + "\",";
    }
    else {
    json += "\"mac\":\"" + String(WiFi.macAddress()) + "\",";
  }
  json += "\"fw\":\"" + String(riot.getVersion()) + "\",";
  json += "\"uptime\":\"" + String(millis() / 1000) + " s" + "\",";

  tempIP = riot.getOwnIP();
  json += "\"ip1\":" + String(tempIP[0]) + ",";
  json += "\"ip2\":" + String(tempIP[1]) + ",";
  json += "\"ip3\":" + String(tempIP[2]) + ",";
  json += "\"ip4\":" + String(tempIP[3]) + ",";

  tempIP = riot.getDestIP();
  json += "\"dip1\":" + String(tempIP[0]) + ",";
  json += "\"dip2\":" + String(tempIP[1]) + ",";
  json += "\"dip3\":" + String(tempIP[2]) + ",";
  json += "\"dip4\":" + String(tempIP[3]) + ",";

  tempIP = riot.getGatewayIP();
  json += "\"gw1\":" + String(tempIP[0]) + ",";
  json += "\"gw2\":" + String(tempIP[1]) + ",";
  json += "\"gw3\":" + String(tempIP[2]) + ",";
  json += "\"gw4\":" + String(tempIP[3]) + ",";

  tempIP = riot.getSubnetMask();
  json += "\"msk1\":" + String(tempIP[0]) + ",";
  json += "\"msk2\":" + String(tempIP[1]) + ",";
  json += "\"msk3\":" + String(tempIP[2]) + ",";
  json += "\"msk4\":" + String(tempIP[3]) + ",";

  json += "\"port\":" + String(riot.getDestPort()) + ",";
  json += "\"power\":" + String(riot.getWifiPower()) + ",";
  // We also read the battery to send it under request
  json += "\"batt\":\"" + String(readBatteryVoltage()) + " volts" + "\",";
  json += "\"id\":" + String(riot.getID()) + ",";
  json += "\"rate\":" + String(motion.getSampleRate());


  json += "}";
  httpServer.send(200, "text/json", json);
  json = String();
}


void handleParams(void) {
  int Rank, ipByte;
  int HttpArgs = httpServer.args();
  String Answer;
  String ArgName, ArgValue;
  IPAddress tempIP;
  char str[32];

  if (HttpArgs) {
    Serial.printf("Received HTTP request with %d args\n", HttpArgs);
    displayArgs(HttpArgs);

    for (int i = 0 ; i < HttpArgs ; i++) {
      // Parsing params withing the submitted URL

      ArgName = httpServer.argName(i);
      ArgValue = httpServer.arg(i);

      if (ArgName == TEXT_SSID)  {
        ArgValue.toCharArray(str, ArgValue.length() + 1);
        riot.setSSID(str);
        Serial.printf("Updated SSID: %s\n", riot.getSSID());
      }
      else if (ArgName == TEXT_PASSWORD) {
        ArgValue.toCharArray(str, ArgValue.length() + 1);
        riot.setPassword(str);
        Serial.printf("Updated wifi password: %s\n", riot.getPassword());
      }
      else if (ArgName == TEXT_DHCP) {
        if (ArgValue == "DHCP")
          riot.setUseDHCP(true);
        else
          riot.setUseDHCP(false);
        Serial.printf("Updated DHCP = %d\n", riot.isDHCP());
        if (!riot.isDHCP())
          Serial.printf("No DHCP : will use static IP address\n");
      }
      else if (ArgName.startsWith("ipi")) {
        Rank = atoi(&ArgName[3]) - 1;
        ipByte = ArgValue.toInt();
        ipByte = constrain(ipByte, 0, 255);
        tempIP = riot.getOwnIP();
        tempIP[Rank] = ipByte;
        riot.setOwnIP(tempIP);
        //Serial.printf("LocalIP[%d] update = %d\n", Rank, ipByte);
      }
      else if (ArgName.startsWith("dip")) {
        Rank = atoi(&ArgName[3]) - 1;
        ipByte = ArgValue.toInt();
        ipByte = constrain(ipByte, 0, 255);
        tempIP = riot.getDestIP();
        tempIP[Rank] = ipByte;
        riot.setDestIP(tempIP);
      }
      else if (ArgName.startsWith("gw")) {
        Rank = atoi(&ArgName[2]) - 1;
        ipByte = ArgValue.toInt();
        ipByte = constrain(ipByte, 0, 255);
        tempIP = riot.getGatewayIP();
        tempIP[Rank] = ipByte;
        riot.setGatewayIP(tempIP);
      }
      else if (ArgName.startsWith("msk")) {
        Rank = atoi(&ArgName[3]) - 1;
        ipByte = ArgValue.toInt();
        tempIP = riot.getSubnetMask();
        tempIP[Rank] = ipByte;
        riot.setSubnetMask(tempIP);
      }
      else if (ArgName == "port") {
        riot.setDestPort(ArgValue.toInt());
        Serial.printf("Updated UDP port = %d\n", riot.getDestPort());
      }
      else if (ArgName == "id") {
        riot.setID(ArgValue.toInt());
        Serial.printf("Updated Module ID# = %d\n", riot.getID());
      }
      else if (ArgName == "rate") {
        motion.setSampleRate(ArgValue.toInt());
        Serial.printf("Updated Module Sample Rate = %d\n", motion.getSampleRate());
      }
      else if (ArgName == "power") {
        wifi_power_t power = (wifi_power_t)ArgValue.toInt();
        riot.setWifiPower(power);
        Serial.printf("Updated Wifi Power to %ddBm\n", power);
      }
    } // End of Browsing Args
  }
  Answer = "Received " + String(HttpArgs) + " args\n";
  Answer += "Saving Parameters and reboot\n";
  httpServer.send(200, "text/plain", Answer);
  Answer = String();

  httpServer.stop();
  storeConfig();
  Serial.printf("Rebooting the module in 2s\n");
  delay(2000);
  reset();

}


void displayArgs(int ArgsNumber) {
  String ArgToDisplay;
  Serial.println("Args List:");
  for (int i = 0 ; i < ArgsNumber ; i++)  {
    ArgToDisplay = httpServer.argName(i) + ":" + httpServer.arg(i);
    Serial.println(ArgToDisplay);
  }
}
