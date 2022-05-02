#!/usr/bin/env python

# based on FIK-6/sw/Telemetry_receiver/habitat_uploader_py3.py
# and GEODOS01/sw/lorasave.py

import paho.mqtt.client as mqtt
import argparse
import logging
import time
import json
import sys
import crc16
import logging
import iso8601
from hashlib import sha256
from datetime import datetime
from base64 import b64encode
import http.client as httplib

callsign = "LetFik7a"
virtual = False;

def checksum(sentence):
    crc = crc16.crc16xmodem(sentence.encode(), 0xffff) # crc16-citt
    return ('{:04X}'.format(crc))

def make_habitat_sentence(sentence):
    sentence = ",".join([str(s) for s in [
        callsign,
        sentence['timestamp'], sentence['lat'], sentence['lon'],
        sentence['altitude'], sentence['course'],
        sentence['speed'], sentence['temperature'],
        sentence['voltage'], sentence['current'],
        sentence['hits']
    ]])
    return "$$" + sentence + "*" + str(checksum(sentence)) + '\n'

def make_request_body(our_name, sentence, callsign):
    date = datetime.utcnow().isoformat("T") + "Z"
    sentence = b64encode(bytes(sentence, encoding="ascii")).decode("ascii")
    return {
        "type": "payload_telemetry",
        "data": {
            "_raw": sentence
            },
        "receivers": {
            our_name: {
                "time_created": date,
                "time_uploaded": date,
                },
            },
    }

def main():
    logging.basicConfig(format='[%(asctime)s] %(levelname)s: %(message)s', level=logging.INFO)
    parser = argparse.ArgumentParser(description='Relay balloon telemetry from ' \
                                                 'The Things Network to Habitat.')
    parser.add_argument('--app_id', '--username', type=str, help='TTN application ID')
    parser.add_argument('--access_key', '--password', type=str, help='TTN access key')
    parser.add_argument('--mqtt_server', '--server', type=str, help='MQTT server address',
                        default='eu1.cloud.thethings.network:8883')
    parser.add_argument('--receiver', default='lora', type=str, help='receiver ID')
    parser.add_argument('--mock', default=False, action='store_true')
    args = parser.parse_args()

    # BUG: it can happen that we upload a sentence containing one
    # of the default zeroes from below. if only there was a way
    # to upload sentences with skipped fields...
    sentence_fields = {
        'timestamp': 0,
        'lat': 0,
        'lon': 0,
        'altitude': 0,
        'course': 0,
        'speed': 0,
        'temperature': 0,
        'voltage': 0,
        'current': 0,
        'hits': 0,
    }

    def process_payload(payload):
        nonlocal sentence_fields

        fields = payload['uplink_message']['decoded_payload']
        ts = iso8601.parse_date(payload['received_at'])

        # wrong timezone --v
        #timestamp = (ts.utcnow() - datetime(1970,1,1)).total_seconds()

        sentence_fields.update({
            'timestamp': datetime.utcnow().strftime("%H:%M:%S"),
        })

        if fields.get('latlon_ok', False):
            sentence_fields.update({
                'lat': fields['lat'],
                'lon': fields['lon'],
            })

        if fields.get('alt_okay', False):
            sentence_fields.update({
                'altitude': fields['alt_m'],
            })

        if fields.get('course_ok', False):
            sentence_fields.update({
                'course': fields['course'],
            })

        if fields.get('speed_ok', False):
            sentence_fields.update({
                'speed': fields['speed_mps'],
            })

        if "V" in fields:
            sentence_fields.update({
                'temperature': fields['°C'],
                'voltage': fields['V'],
                'current': fields['mA'] / 1000,
                'hits': fields['hits'],
            })

        if not fields.get('latlon_ok', False):
            logging.info('updated subset of sentence, postponing upload until lat/lon is OK')
            return

        sentence = make_habitat_sentence(sentence_fields)
        logging.info("uploading %s", sentence.strip())
        addr = "/habitat/_design/payload_telemetry/_update/add_listener/%s" % sha256(b64encode(sentence.encode())).hexdigest()
        body = bytes(json.dumps(make_request_body(args.receiver,
                                sentence, callsign)), encoding="ascii")
        try:
            c = httplib.HTTPConnection("habitat.habhub.org") # DB uploader
            c.request("PUT", addr, body,
                { b"Content-Type": b"application/json", b'Accept': b"application/json" }
            )
            resp = c.getresponse()
            logging.info("PUT: %d %s %s", resp.status, resp.reason, resp.read())
        except Exception as e:
            logging.error("request to habhub: %s", e)

    if args.mock:
        process_payload(mockm)
        return

    def callback(_client, _userdata, msg):
        if not msg.topic.endswith("/up"):
            logging.info("message in topic %s: %s" % (msg.topic, msg.payload))
            return
        else:
            try:
                process_payload(json.loads(msg.payload))
            except Exception as e:
                logging.error(e)

            try:
                logging.info("picked up uplink message from %s", payload["end_device_ids"]["device_id"])
            except:
                logging.info("picked up weird uplink message")

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            logging.info("connected")
            client.subscribe("#")
        else:
            logging.error("connection attempt failed (code: %d)" % rc)

    client = mqtt.Client()
    client.on_message = callback
    client.on_connect = on_connect
    client.tls_set()
    addr_pieces = args.mqtt_server.split(":", 2)
    host = addr_pieces[0]
    port = int(addr_pieces[1] if len(addr_pieces) == 2 else "8883")
    client.username_pw_set(args.app_id, args.access_key)
    client.connect(host, port, 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
