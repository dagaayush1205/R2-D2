import serial 
import rerun as rr
from cobs import cobs
import zlib 
import ctypes 
import sys 
import time

class GPSMssg(ctypes.Structure):
    _fields_ = [ 
     ("latitude", ctypes.c_int64),
     ("longitude", ctypes.c_int64),
     ("altitude", ctypes.c_int32),
     ("bearing", ctypes.c_int32),
     ("crc", ctypes.c_uint32),
    ]

# rr.init("GPS DATA", spawn=True)
# rr.connect_tcp("0.0.0.0:9876")

def process_cobs(encoded_msg):
    if encoded_msg[-1] != 0:
        print("Error: msg does not have end byte")
        return 

    try:
        decoded_msg = cobs.decode(encoded_msg[:-2])
        gps_msg = ctypes.cast(decoded_msg, ctypes.POINTER(GPSMssg))

        # if gps_msg.contents.crc != zlib.crc32(decoded_msg[:28]):
        #     print("Error: Mssg is corrupt crc did not match")
        #     return 
            
        return {
            "latitude": gps_msg.contents.latitude,
            "longitude": gps_msg.contents.longitude,
            "height": gps_msg.contents.altitude,
            "heading": gps_msg.contents.bearing
        }
    except Exception as e: 
        print(f"Error while processing data:{e}")

def main():
    port = sys.argv[1]
    ser = serial.Serial(port, baudrate=9600, timeout=1, exclusive=False)

    location_data = list()
    
    while True: 
        try: 
            raw_data = ser.read(ctypes.sizeof(GPSMssg))
            result = process_cobs(raw_data + b'\x00')
            
            if result: 
                print("GPS data : ", result)
                location_data.append([result["latitude"], result["longitude"]])
                # rr.log(
                #     "geo_points", 
                #     rr.GeoLineStrings(lat_lon=location_data),
                #     radii=rr.Radius.ui_points(2.0), 
                #     colors=[0, 0, 255]
                # )
                # rr.log(
                #     "heading", 
                #     rr.TextLog(str(result["heading"]), level=rr.TextLogLevel.INFO)
                # )
        except Exception as e: 
            print(f"Error: {e}")
            
        time.sleep(1)

if __name__ == "__main__":
    main()
