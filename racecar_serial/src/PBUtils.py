from binascii import unhexlify
import threading
import time

# Serialization utils
class PBSerializationHandler:
    def __init__(self, msg_obj):
        self._msg_obj = msg_obj

    def encode_msgs(self, ids, msgs):
        msg = "<"

        for id_msg, pb_msg in zip(ids, msgs):
            msg += str(id_msg) + "|"
            for byte in bytearray(pb_msg.SerializeToString()):
                msg += str(hex(byte))[2:].zfill(2)  # Remove \x and fill with 0 in front to always takes 2 digits
            msg += ";"

        msg += ">"
        return msg

    def encode_msg(self, id, msg):
        return self.encode_msgs([id], [msg])

    def deserialize(self, messages):
        messages = messages.decode("ascii")
        msg_array = messages[1:-1].split(';')      # Remove < > characters and split sub-msgs

        object_list = []

        for msg in msg_array:
            if len(msg) > 0:
                msg_id, raw_msg = msg.split("|")    # Find the id of the message
                msg_id = int(msg_id)
                obj = self._msg_obj[msg_id]
                obj.ParseFromString(unhexlify(raw_msg))
                object_list.append([msg_id, obj])

        return object_list



# Serial communication utils

class ArduinoReadHandler(threading.Thread):
    def __init__(self, sleeptime, readfunc):
        self._sleeptime = sleeptime
        self._readfunc = readfunc
        threading.Thread.__init__(self)
        self._runflag = threading.Event()
        self._runflag.clear()
        self._run = True

    def run(self):
        self._runflag.set()
        self.worker()

    def worker(self):
        while self._run:
            if self._runflag.is_set():
                self._readfunc()
            time.sleep(self._sleeptime)

    def pause(self):
        self._runflag.clear()

    def resume(self):
        self._runflag.set()

    def running(self):
        return self._runflag.is_set()

    def kill(self):
        self._run = False


class PBSerialHandler:
    def __init__(self, serial, callback, msg_obj, sleeptime=0.01):
        self._serial = serial
        self._sleeptime = float(sleeptime)
        self._callback = callback

        self._interlock = False
        self._response = None

        self._serialization_handler = PBSerializationHandler(msg_obj)
        self._worker = ArduinoReadHandler(self._sleeptime, self.read_callback)
        self._worker.start()

    def kill(self):
        self._worker.kill()

    def read_callback(self):
        if not self._interlock:
            self._interlock = True
            try:
                input = self._serial.read()
                if input == b'<':
                    buffer = self._serial.read_until(b'>')
                    self._serial.flush()
                    self._response = b'<' + buffer
                    self._callback(self._response)
            except Exception as e:
                print("Read call back error " + str(e))

            self._interlock = False

    def write_pb_msg(self, id, msg):
        self.write_pb_msgs([id], [msg])

    def write_pb_msgs(self, ids, msgs):
        encoded_msg = self._serialization_handler.encode_msgs(ids, msgs)

        while self._interlock:
            time.sleep(self._sleeptime)

        self._interlock = True
        self._serial.write(encoded_msg.encode("ascii"))
        self._serial.flush()
        self._interlock = False
