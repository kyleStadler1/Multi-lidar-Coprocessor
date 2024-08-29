
const unsigned int TIMEOUT_MS = 12;
const unsigned int BAUD = 115200;
const unsigned int STRENGTH_THRESH = 100;
const uint8_t DATA_FRAME_LEN = 9;
const uint8_t INTAKE_MAX_DIST = 50; //ask cad

#define coneLidar Serial4
#define cubeLidar Serial5
#define rioSerial Serial1
#define HEADER_VAL 0x59
//byte # of:
#define HEADER0 0 
#define HEADER1 1
#define DIST_L 2
#define DIST_H 3
#define STRENGTH_L 4
#define STRENGTH_H 5
#define CHECKSUM 8

#define DEBUG 1

struct lidarData { //struct to hold decoded info from each read
  uint16_t dist = 0;
  uint16_t strength = 0;
  bool valid = 0;
  bool con = 0;
};

uint8_t dataOutbuf[4] = {0x59, 0xff, 0xff, 0x0};

lidarData coneData;
lidarData cubeData;


void setup() {
  Serial.begin(9600);
  coneLidar.begin(BAUD);
  coneData.con = initLidar(&coneLidar);
  
  cubeLidar.begin(BAUD);
  cubeData.con = initLidar(&cubeLidar);
  rioSerial.begin(BAUD);
//  while(!rioSerial.available()) {
//    
//    delay(100);
//    Serial.println("init");
//  }
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  coneData.con = getDataFrame(&coneData, &coneLidar);
  coneData.valid = coneData.con && validateData(&coneData);
  
  cubeData.con = getDataFrame(&cubeData, &cubeLidar);
  cubeData.valid = cubeData.con && validateData(&cubeData);

  if (!coneData.con) {
    clearSerialBuf(&coneLidar);
    coneLidar.begin(BAUD);
    initLidar(&coneLidar);
  }
  if (!cubeData.con) {
    clearSerialBuf(&cubeLidar);
    cubeLidar.begin(BAUD);
    initLidar(&cubeLidar);
  }

  
  if (coneData.valid) {
    dataOutbuf[1] = coneData.dist;
  } else {
    dataOutbuf[1] = 0xff;
  }
  if (cubeData.valid) {
    dataOutbuf[2] = cubeData.dist;
  } else {
    dataOutbuf[2] = 0xff;
  }
  dataOutbuf[3] = dataOutbuf[0] + dataOutbuf[1] + dataOutbuf[2];
  Serial.println(arrToString(dataOutbuf, 4));
  Serial.println(rioSerial.available());
  if (rioSerial.available() && rioSerial.read() == 0x59) {
    Serial.println("got");
    for (uint8_t i = 0; i < 4; i++) {
      rioSerial.write(dataOutbuf[i]);
      //Serial.println(dataOutbuf[i]);
    }
    clearSerialBuf(&rioSerial);
  }
}

bool getDataFrame(lidarData* data_, HardwareSerial *port) {
  uint8_t buf[DATA_FRAME_LEN];
  uint8_t i = 0;
  uint8_t sum = 0xB2; //pre calculates 0x59 + 0x59
  uint8_t val = 0;
  requestFrame(port);
  while (i < DATA_FRAME_LEN) {
    if (i < 2 && getByte(&val, port) && val == HEADER_VAL) { //if we havent gotten both 0x59's it will continue this logic branch
      buf[i++] = val;
    } else if (i > HEADER1 && getByte(&val, port)){ //this is after we got both 0x59's
      if (i != CHECKSUM) { //adds to rolling sum except for last byte sinc we need to compare it to that, not add it
        sum += val;
      }
      buf[i++] = val;
    } else { //we get here if we are still looking for header and it timeouts OR if we got header but we timeout while getting data byte
      return false;
    }
  }
  if (sum == buf[CHECKSUM]) { //checks if our rolling sum == to last piece of data sent to verify that there was no jumbling of bits when transmitting
    data_->dist = buf[DIST_L] + (buf[DIST_H] << 8);
    data_->strength = buf[STRENGTH_L] + (buf[STRENGTH_H] << 8);
    return true;
  }
  return false; //low k should never get here
}

bool validateData(lidarData* data_) { //high level data validaito, brike out if statemtnts for readability 
  if (data_->strength < STRENGTH_THRESH) {
    return false;
  }
  if (data_->dist > 65532) {
    return false;
  }
  if (data_->dist > INTAKE_MAX_DIST) {
    return false;
  }
  return true;
}

bool getByte(uint8_t *val, HardwareSerial *port) { //params: addr of data to write to, addr of hardware port to read from
  unsigned long startTime_ms = millis();
  while(!port->available()) {
    if (millis() - startTime_ms > TIMEOUT_MS) { //if read is taking longer than TIMEOUT_MS, we stop trying and give up
      //Serial.println("SERIAL TIMEOUT");
      return false;
    }
  }
  *val = port->read(); //assign the read value to the var addr given to us
  return true;
}

bool clearSerialBuf(HardwareSerial *port) {
  size_t len = port->available();
  for (size_t i = 0; i < len; i++) {
    port->read();
  }
  return (!port->available());
}

bool initLidar(HardwareSerial *port) {
  uint8_t header = 0x5a;
  uint8_t len = 0x06;
  uint8_t id = 0x03;
  uint8_t ll = 0x0;
  uint8_t hh = 0x0;
  uint8_t checksum = header + len + id + ll + hh;

  uint8_t bufOut[len] = {header, len, id, ll, hh, checksum};

  for (uint8_t i = 0; i < len; i++) {
    port->write(bufOut[i]);
  }
  uint8_t bufIn[len];
  for (uint8_t i = 0; i < len; i++) {
    uint8_t val;
    if (!getByte(&val, port)) {
      return false;
    }
    bufIn[i] = val;
  }
  for (uint8_t i = 0; i < len; i++) {
    if (bufIn[i] != bufOut[i]) {
      return false;
    }
  }
  return true;
}

void requestFrame(HardwareSerial *port) {
  uint8_t len = 4;
  uint8_t bufOut[len] = {0x5a, 0x04, 0x04, 0x62};
  for (uint8_t i = 0; i < len; i++) {
    port->write(bufOut[i]);
  }
}

String arrToString(uint8_t arr[], uint8_t len) { //helper util function
  String str = "";
  for (size_t i = 0; i < len; i++) {
    str += "[" + String(i) + "]: " + String(arr[i], DEC) + "\t";
  }
  return str;
}
