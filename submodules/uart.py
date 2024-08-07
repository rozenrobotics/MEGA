from serial import Serial 


class UART:
    def __init__(
            self,
            port: str = '/dev/ttyACM0',
            baud: int = 115200,
            ) -> None:
        self.port = port 
        self.baud = baud

        self.uart = Serial(
                port=self.port,
                baudrate=self.baud,
                )
        self.recv_pkg: str = None
        self.send_pkg: str = None

        self.fl_sent: bool = False
        self.fl_recv: bool = False

    def send(self) -> None:
        self.uart.write(self.send_pkg.encode('utf-8'))
        self.fl_sent = True


    def recv(self) -> None:
        ...


amega: UART = UART(
        port='/dev/ttyACM0',
        baud=115200,
        )

cnt = 0
while True:
    cnt = 0 if cnt > 100 else cnt + 1

    amega.send_pkg = str(cnt)
    amega.send()



        


