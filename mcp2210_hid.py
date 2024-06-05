import time
import hid


class DataBlock():

    def __init__(self):
        pass


class MCP2210():

    @staticmethod
    def find_MCP2210():
        for device_dict in hid.enumerate():
            if "MCP2210" in device_dict["product_string"]:
                keys = list(device_dict.keys())
                keys.sort()
                info = ""
                for key in keys:
                    info += f"{key}: {device_dict[key]}\n"
                return info.strip()
        return "MCP2210 not found"

    def __init__(self):
        self.dev = hid.device()
        self.init_para_SPI()
        self.init_para_chip()

    @staticmethod
    def big_endian(value, length):
        return list(value.to_bytes(length, "big"))

    @staticmethod
    def de_big_endian(list_data):
        data_byte = bytes(list_data)
        return int.from_bytes(data_byte, "big")

    @staticmethod
    def little_endian(value, length):
        return list(value.to_bytes(length, "little"))

    @staticmethod
    def de_little_endian(list_data):
        data_byte = bytes(list_data)
        return int.from_bytes(data_byte, "little")

    def init_para_chip(self):
        self.gpio_design = 8 * [0]  # GPIO:0, CS:1, func:2
        self.gp8_design = GP8Design.INPUT  # input:0, func:2
        self.gpio_out = 0xaa
        self.gpio_direction = 0xff  # input: 1, output: 0
        self.remote_wakeup_enable = 1
        self.interrupt_mode = InterruptMode.COUNTFALLINGEDGE
        self.spi_bus_release_enable = 0
        self.access_control = 0
        # self.password = 0

        return self

    def gen_chip_setting(self):
        # generate 4 ~ 26 byte of SPI setting cmd
        cmd = []
        cmd += self.gpio_design  # 4~11
        cmd += [self.gp8_design]  # 12
        cmd += [self.gpio_out, 0xff]  # 13~14
        cmd += [self.gpio_direction, 0xff]  # 15~16
        byte17 = self.remote_wakeup_enable << 4
        byte17 += self.interrupt_mode << 1
        byte17 += self.spi_bus_release_enable
        cmd += [byte17, self.access_control]  # 17~18
        # cmd += self.password[:8].encode()  # 19~26
        return cmd

    def set_chip_setting_pwrup(self):
        cmd = [PwrUpCmd.SET, PwrUpSubCmd.CHIPSET, 0, 0]
        cmd += self.gen_chip_setting()
        receive = self.query(cmd)
        if receive[0:3] == [cmd[0], 0x00, cmd[1]]:
            return self
        elif receive[0:2] == [cmd[0], 0xFB]:
            raise Exception(ERRORMSG.BLOCKEDACCESS)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)
        return self

    def set_chip_setting(self):
        cmd = [VMCmd.SETCHIPSET, 0, 0, 0]
        cmd += self.gen_chip_setting()[:14]
        receive = self.query(cmd)
        if receive[0:2] == [cmd[0], 0x00]:
            return self
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_chip_setting_pwrup(self):
        cmd = [PwrUpCmd.GET, PwrUpSubCmd.CHIPSET, 0, 0]
        receive = self.query(cmd)
        if receive[0:3] == cmd[0:1]+[0, 0x20]:
            return self.analyze_chip_setting(receive)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_chip_setting(self):
        cmd = [VMCmd.GETCHIPSET, 0, 0, 0]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return self.analyze_chip_setting(receive)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def analyze_chip_setting(self, receive):
        para_chip = DataBlock()
        para_chip.gpio_design = receive[4:12]
        para_chip.gp8_design = receive[12]
        para_chip.gpio_out = receive[13]
        para_chip.gpio_direction = receive[15]
        para_chip.remote_wakeup_enable = (receive[17] >> 4) & 1
        para_chip.interrupt_mode = (receive[17] >> 1) & 0b111
        para_chip.spi_bus_release_enable = receive[17] & 1
        para_chip.access_control = receive[18]
        return para_chip

    def init_para_SPI(self):
        self.bitrate = 3 * 10**6  # 1.5k~3M
        self.cs_idle = 0xff
        self.cs_active = 0x00
        self.delay_cs_data = 0  # quanta of 100us
        self.delay_data_cs = 0  # quanta of 100us
        self.delay_data_data = 0  # quanta of 100us
        self.datalen_spi = 32  # bytes
        self.spimode = 0

        self.interval_retry = 0.005  # sec
        return self

    def gen_SPI_setting(self):
        # generate 4 ~ 20 byte of SPI setting cmd
        cmd = []
        cmd += self.little_endian(self.bitrate, 4)  # 4~7
        cmd += [self.cs_idle, 0, self.cs_active, 0]  # 8~11
        # 12~19
        cmd += self.little_endian(self.delay_cs_data, 2)
        cmd += self.little_endian(self.delay_data_cs, 2)
        cmd += self.little_endian(self.delay_data_data, 2)
        cmd += self.little_endian(self.datalen_spi, 2)
        cmd += [self.spimode]  # 20
        return cmd

    def set_SPI_setting_pwrup(self):
        cmd = [PwrUpCmd.SET, PwrUpSubCmd.SPISET, 0, 0]
        cmd += self.gen_SPI_setting()
        receive = self.query(cmd)
        if receive[0:3] == [cmd[0], 0x00, cmd[1]]:
            return self
        elif receive[0:2] == [cmd[0], 0xFB]:
            raise Exception(ERRORMSG.BLOCKEDACCESS)
        elif receive[0:2] == [cmd[0], 0xF8]:
            raise Exception(ERRORMSG.NOTWRITTEN)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def set_SPI_setting(self):
        cmd = [VMCmd.SETSPISET, 0, 0, 0]
        cmd += self.gen_SPI_setting()
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return self
        elif receive[0:2] == [cmd[0], 0xf8]:
            raise Exception(ERRORMSG.NOTWRITTEN)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_SPI_setting_pwrup(self):
        cmd = [PwrUpCmd.GET, PwrUpSubCmd.SPISET, 0, 0]
        receive = self.query(cmd)
        if receive[0:3] == cmd[0:1] + [0, 0x10]:
            return self.analyze_SPI_setting(receive)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_SPI_setting(self):
        cmd = [VMCmd.GETSPISET, 0, 0, 0]
        receive = self.query(cmd)
        if receive[0:3] == cmd[0:2]+[0x11]:
            return self.analyze_SPI_setting(receive)
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def analyze_SPI_setting(self, receive):
        para_SPI = DataBlock()
        para_SPI.bitrate = self.de_little_endian(receive[4:8])
        para_SPI.cs_idle = receive[8]
        para_SPI.cs_active = receive[10]
        para_SPI.delay_cs_data = self.de_little_endian(
                receive[12:14])
        para_SPI.delay_data_cs = self.de_little_endian(
                receive[14:16])
        para_SPI.delay_data_data = self.de_little_endian(
                receive[16:18])
        para_SPI.datalen_spi = self.de_little_endian(receive[18:20])
        para_SPI.spimode = receive[20]
        return para_SPI

    def open(self):
        self.dev.open(1240, 222)
        return self

    def initiate(self):
        self.init_para_chip()
        self.init_para_SPI()
        self.set_chip_setting()
        self.set_SPI_setting()
        return self

    def close(self):
        self.dev.close()
        return self

    def write(self, cmd: list):
        tail = (64 - len(cmd)) * [0]
        self.dev.write([0] + cmd + tail)
        return self

    def read(self, size=64):
        return self.dev.read(size)

    def readprint(self, size=64):
        receive = self.read(size)
        for index, val in enumerate(receive):
            print(f"{index}: {val:#04x} {val:>3}")
        return receive

    def query(self, cmd, readsize=64):
        self.write(cmd)
        return self.read(64)

    def queryprint(self, cmd, readsize=64):
        self.write(cmd)
        return self.readprint(64)

    def set_gpio_direction_all(self, direction: int):
        # input:1, output:0
        # MSB: GPIO7, LSB: GPIO0
        direction &= 0xff
        cmd = [VMCmd.SETGPIODIRECTION]
        cmd += [0, 0, 0, direction]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return self
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_gpio_direction_all(self):
        # input:1, output:0
        # MSB: GPIO7, LSB: GPIO0
        cmd = [VMCmd.GETGPIODIRECTION, 0]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return receive[4] & 0xff
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def set_gpio_direction(self, gpio: int, direction: int):
        # input:1, output:0
        direction &= 0x01
        mask = 1 << (gpio & 0x07)

        direction_current = self.get_gpio_direction_all()
        if direction:
            direction_new = direction_current | mask
        else:
            direction_new = direction_current & ~mask
        self.set_gpio_direction_all(direction_new)
        return self

    def get_gpio_direction(self, gpio):
        # input:1, output:0
        gpio &= 0x07
        direction_current = self.get_gpio_direction_all()
        return (direction_current >> gpio) & 0x01

    def set_gpio_value_all(self, value: int):
        # ON: 1, OFF: 0
        # MSB: GPIO7, LSB: GPIO0
        value &= 0xff
        cmd = [VMCmd.SETGPIOVALUE]
        cmd += [0, 0, 0, value]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            self.gpio_value_set = value
            return receive[4] & 0xff  # GPIO Actual Value
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def get_gpio_value_all(self):
        # MSB: GPIO7, LSB: GPIO0
        cmd = [VMCmd.GETGPIOVALUE, 0]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return receive[4] & 0xff
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def set_gpio_value(self, gpio: int, value: int):
        gpio &= 0x07
        value &= 0x01
        mask = 1 << gpio

        value_current = self.get_gpio_value_all()
        if value:
            value_new = value_current | mask
        else:
            value_new = value_current & ~mask
        value_actual = self.set_gpio_value_all(value_new)
        return (value_actual >> gpio) & 0x01

    def get_gpio_value(self, gpio: int):
        gpio &= 0x07
        value_current = self.get_gpio_value_all()
        return (value_current >> gpio) & 0x01

    def get_gpio_value_8(self):
        cmd = [VMCmd.GETGPIOVALUE, 0]
        receive = self.query(cmd)
        if receive[0:2] == cmd[0:2]:
            return receive[5] & 1
        else:
            raise Exception(ERRORMSG.CMDUNMATCH)

    def xfer_spi_data(self, data):
        if self.datalen_spi != len(data):
            self.datalen_spi = len(data)
            self.set_SPI_setting()

        data_send, data_rest = data[:60], data[60:]
        data_spi_rx = []

        while True:
            cmd = [SPICmd.TRANSFER, len(data_send), 0, 0]
            cmd += data_send
            receive = self.query(cmd)

            if receive[0:2] == [cmd[0], 0xf7]:  # RES1
                raise Exception("SPI bus not available")
            if receive[0:2] == [cmd[0], 0xf8]:  # RES3
                time.sleep(self.interval_retry)
                continue

            if receive[0:2] == [cmd[0], 0x00]:  # RES2, 4, 5
                data_send, data_rest = data_rest[:60], data_rest[60:]
                if receive[3] == 0x10:  # RES5
                    data_spi_rx += receive[4:4+receive[2]]
                    break
                elif receive[3] == 0x20:  # RES2
                    continue
                elif receive[3] == 0x30:  # RES4
                    data_spi_rx += receive[4:4+receive[2]]
                    continue
            else:
                raise Exception(ERRORMSG.CMDUNMATCH)  # else

        if len(data) != len(data_spi_rx):
            msg = "SPI send data lenght and receive length not equal"
            raise Exception(msg)

        return data_spi_rx

    def xfer_spi(self, data, *chips, csmask=0xff):
        if chips:
            cs = 0
            for chip in chips:
                cs += 1 << chip
            cs = (~cs) & 0xff
        else:
            cs = (~csmask) & 0xff

        self.cs_active = cs
        self.set_SPI_setting()

        return self.xfer_spi_data(data)


class PwrUpCmd():
    SET = 0x60
    GET = 0x61
    UNLOCK = 0x70


class PwrUpSubCmd():
    CHIPSET = 0x20
    SPISET = 0x10
    USBKEYPARAS = 0x30
    USBPRODUCTNAME = 0x40
    USBMANUFACTURENAME = 0x50


class VMCmd():
    GETCHIPSET = 0x20
    SETCHIPSET = 0x21
    SETSPISET = 0x40
    GETSPISET = 0x41
    SETGPIOVALUE = 0x30
    GETGPIOVALUE = 0x31
    SETGPIODIRECTION = 0x32
    GETGPIODIRECTION = 0x33


class EEPROMCmd():
    READ = 0x50
    WRITE = 0x51


class GP6Cmd():
    GETEVENTNUM = 0x12


class SPICmd():
    TRANSFER = 0x42
    CANCEL = 0x11
    BUSRELEASE = 0x80


class StatusCmd():
    GETSTATUS = 0x10


class GPIODirection():
    INPUT = 1
    OUTPUT = 0


class GPIOOutput():
    OFF = 0
    ON = 1


class GPIODesign():
    GPIO = 0x00
    CS = 0x01
    FUNC = 0x02


class GP8Design():
    INPUT = 0x00
    FUNC = 0x02


class InterruptMode():
    RESERVED0 = 0b111
    RESERVED1 = 0b110
    RESERVED2 = 0b101
    COUNTHIGHPULSE = 0b100
    COUNTLOWPULSE = 0b011
    COUNTRISINGEDGE = 0b010
    COUNTFALLINGEDGE = 0b001
    NOCOUNT = 0b000


class NVRAMAAccessCtrl():
    UNLOCK = 0x00
    PASSWORD = 0x40
    LOCK = 0x80


class ERRORMSG():
    CMDUNMATCH = "Something wrong during HID communication"
    BLOCKEDACCESS = "Blocked Access.\n"
    BLOCKEDACCESS += "Password is wrong, "
    BLOCKEDACCESS += "or the settings are permanantely locked"
    NOTWRITTEN = "Settings not written"


if __name__ == "__main__":
    x = MCP2210()
    x.open()
    x.gpio_design = 8*[1]
    x.set_chip_setting()
    x.set_gpio_direction_all(0xff)
    print(x.xfer_spi([0xf, 0xf0], 4))
