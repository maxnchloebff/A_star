import serial


class Ser():
    def __init__(self):
        self.port = serial.Serial(port='COM7', baudrate=115200, bytesize=8, parity='E', stopbits=1, timeout=10)

    def send_cmd(self, cmd):
        self.port.write(cmd)
        response = self.port.readall()
        response = self.convert_hex(response)
        return response

    def convert_hex(self, string):
        res = []
        result = []
        for item in string:
            res.append(item)
        for i in res:
            result.append(hex(i))

        return result


robot_serial = Ser()
send_cmd1 = [0xff, 0xb0, 0x01, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x31]
send_cmd2 = [0xff, 0xb0, 0x04, 0x05, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85]
send_cmd3 = [0xff, 0x00, 0x01, 0x01, 0x06, 0x40, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00]

a = robot_serial.send_cmd(send_cmd1)
while a == '':
    a = robot_serial.send_cmd(send_cmd1)

b = robot_serial.send_cmd(send_cmd2)
print(b)

c = robot_serial.send_cmd(send_cmd3)
while c == '':
    c = robot_serial.send_cmd(send_cmd3)