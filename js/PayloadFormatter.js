function Decoder(b, port) {
  if (b[0] == 0xf0) {
    var temperature;
    var current;
    var battery_voltage = b[1] + 175;
    var current_sign = b[3] & 1;
    var battery_current = b[2];
    if (current_sign) {
      current = 0xFFFFFF00 | battery_current;  // fill in most significant bits with 1's
    }
    else {
      current = battery_current;
    }
      
    var voltage = battery_voltage / 100;
    
    var temp = b[3] >> 1;
    var temp_sign = b[3] >> 7;
    if (temp_sign) {
      temperature = 0xFFFFFF80 | temp;  // fill in most significant bits with 1's
    }
    else {
      temperature = temp;
    }
    
    var hits = b[4] + (b[5] << 8);
    
    return {
      'V' : voltage,
      'mA' : current,
      '°C' : temperature,
      'hits' : hits
    };
  }
  else {
    var lat = (b[1]<<24)|(b[2]<<16)|(b[3]<<8)|b[4];
    var lon = (b[5]<<24)|(b[6]<<16)|(b[7]<<8)|b[8];
    var latlon_age = (b[9]<<8)|b[10];
    var alt = (b[11]<<8)|b[12];
    var course = (b[13]<<8)|b[14];
    var speed = (b[15]<<8)|b[16];
    
    var decoded = {
      "lat": lat/4194304.0,
      "lon": lon/4194304.0,
      "latlon_ok": b[0]&0x01,
      "latlon_age_s": latlon_age,
      "alt_m": alt,
      "alt_okay": (b[0]>>1)&0x01,
      "course": course/64.0,
      "course_ok": (b[0]>>3)&0x01,
      "speed_mps": speed/16.0,
      "speed_ok": (b[0]>>4)&0x01
    };
    return decoded;
  }
}

