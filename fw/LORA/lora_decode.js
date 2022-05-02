function parseHex(hex)
{
  var r = [];
  for (var i = 0; i < hex.length; i += 2) {
    r.push(parseInt(hex.substr(i, 2), 16));
  }
  return r;
}

function decode(b) {
  var lat = (b[1]<<24)|(b[2]<<16)|(b[3]<<8)|b[4];
  var lon = (b[5]<<24)|(b[6]<<16)|(b[7]<<8)|b[8];
  var latlon_age = (b[9]<<8)|b[10];
  var alt = (b[11]<<8)|b[12];
  var course = (b[13]<<8)|b[14];
  var speed = (b[15]<<8)|b[16];
  
  var decoded = {
    "lat": lat/4194304.0,
    "lon": lon/4194304.0,
    "latlon_ok": b[0]&0x01 !== 0,
    "latlon_age_s": latlon_age,
    "alt_m": alt,
    "alt_okay": b[0]&0x02 !== 0,
    "course": course/64.0,
    "course_ok": b[0]&0x04 !== 0,
    "speed_mps": speed/16.0,
    "speed_ok": b[0]&0x08 !== 0
  };

  return decoded;
}

if (require.main === module && process.argv.length >= 2)
  console.log(decode(parseHex(process.argv[2])));
