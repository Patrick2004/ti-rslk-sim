"""Nokia 5110 (PCD8544) 84x48 LCD on an SPI port."""


class Nokia5110:
    W, H = 84, 48

    def __init__(self, machine, spi, dc_pin, rst_pin):
        self.m = machine
        self.dc_pin = dc_pin
        self.rst_pin = rst_pin
        self.ram = bytearray(self.W * 6)
        self.x = self.y = 0
        self.ext = False          # H bit: extended instruction set
        self.vertical = False
        self.mode = 0             # 0 blank, 1 normal, 2 all on, 3 inverse
        self.used = False         # becomes True once the program talks to the LCD
        spi.spi_device = self

    def _in_reset(self):
        g = self.m.gpio
        port, bit = self.rst_pin
        return bool(g.dir[port] & (1 << bit)) and not g.out[port] & (1 << bit)

    def spi_byte(self, b, t):
        if self._in_reset():
            self.ram[:] = bytes(len(self.ram))
            self.x = self.y = 0
            self.mode = 0
            return 0xFF
        self.used = True
        if self.m.gpio.is_output_high(*self.dc_pin):          # data
            self.ram[self.y * self.W + self.x] = b
            if self.vertical:
                self.y += 1
                if self.y >= 6:
                    self.y = 0
                    self.x = (self.x + 1) % self.W
            else:
                self.x += 1
                if self.x >= self.W:
                    self.x = 0
                    self.y = (self.y + 1) % 6
            return 0xFF
        if b & 0xF8 == 0x20:                                  # function set
            self.ext = bool(b & 1)
            self.vertical = bool(b & 2)
        elif not self.ext:
            if b & 0xF8 == 0x08:
                self.mode = {0x08: 0, 0x0C: 1, 0x09: 2, 0x0D: 3}.get(b & 0x0D, 1)
            elif b & 0xF8 == 0x40:
                self.y = min(b & 7, 5)
            elif b & 0x80:
                self.x = min(b & 0x7F, self.W - 1)
        return 0xFF

    def pixel(self, x, y):
        on = bool(self.ram[(y >> 3) * self.W + x] >> (y & 7) & 1)
        if self.mode == 0:
            return False
        if self.mode == 2:
            return True
        return (not on) if self.mode == 3 else on
