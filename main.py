import serial
import time
import cv2

def configure_ports():
    CLIPORT = "/dev/ttyUSB0"
    CLI_baudrate = 115200

    DATAPORT = "/dev/ttyUSB1"
    DATA_baudrate = 921600
    
    configFileName = "xwr68xxconfig.cfg"
    
    CLI_ser = serial.Serial(CLIPORT, CLI_baudrate)
    DATA_ser = serial.Serial(DATAPORT, DATA_baudrate)
    
    with open(configFileName, "r") as configFile:
        config = [line.rstrip('\r\n') for line in configFile]
        for i in config:
            CLI_ser.write((i+"\n").encode())
            print(i)
            time.sleep(0.01)
    return CLI_ser, DATA_ser

def read_data(CLI_ser, DATA_ser):
    
    CLI_bytecount = CLI_ser.inWaiting()
    DATA_bytecount = DATA_ser.inWaiting()
    
    raw_CLI = CLI_ser.read(CLI_bytecount)
    raw_DATA = DATA_ser.read(DATA_bytecount)
    
    return raw_CLI, raw_DATA

def check_detected(CLI_ser, DATA_ser):
    d1, d2 = read_data(CLI_ser, DATA_ser)
    print(d1)
    print(d2)
    return len(d1) > 0 or len(d2) > 0

def main():
    counter = 0
    img_folder = "./detections"
    cap = cv2.VideoCapture(0, CAP.DSHOW)
    CLI_ser, DATA_ser = configure_ports()
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Unsuccessful capture")
            break
        try:
            counter += 1
            status = check_detected(CLI_ser, DATA_ser)
            print(status)
            if status:
                cv2.imwrite(f"{img_folder}/img{counter}.png", frame)
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("Program exited")
            break
        except Exception as e:
            print(e)
            break
    CLI_ser.close()
    DATA_ser.close()
    cap.release()
    
if __name__ == "__main__":
    main()