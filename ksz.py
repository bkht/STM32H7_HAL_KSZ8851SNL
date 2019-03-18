# Translates register value to byte sequence and back for KSZ8851SNL
import sys

#print 'Number of arguments:', len(sys.argv), 'arguments.'
#print 'Argument List:', str(sys.argv)

def get_name(reg):
    name = ""
    if (reg == 0x10):
        name = "MARL"
    if (reg == 0x12):
        name = "MARM"
    if (reg == 0x14):
        name = "MARH"
    if (reg == 0x20):
        name = "OBCR"
    if (reg == 0x26):
        name = "GRR"
    if (reg == 0x70):
        name = "TXCR"
    if (reg == 0x74):
        name = "RXCR1"
    if (reg == 0x76):
        name = "RXCR2"
    if (reg == 0x78):
        name = "TXMIR"
    if (reg == 0x7C):
        name = "RXFHSR"
    if (reg == 0x7E):
        name = "RXFHBCR"
    if (reg == 0x80):
        name = "TXQCR"
    if (reg == 0x82):
        name = "RXQCR"
    if (reg == 0x84):
        name = "TXFDPR"
    if (reg == 0x86):
        name = "RXFDPR"
    if (reg == 0x90):
        name = "IER"
    if (reg == 0x92):
        name = "ISR"
    if (reg == 0x9C):
        name = "RXFCTR"
    if (reg == 0x9E):
        name = "TXNTFSR"
    if (reg == 0xB0):
        name = "FCLWR"
    if (reg == 0xB2):
        name = "FCHWR"
    if (reg == 0xC0):
        name = "CIDER"
    if (reg == 0xC8):
        name = "IACR"
    if (reg == 0xD0):
        name = "IADLR"
    if (reg == 0xD2):
        name = "IADHR"
    if (reg == 0xD4):
        name = "PMECR"
    if (reg == 0xD8):
        name = "PHYRR"
    if (reg == 0xE4):
        name = "P1MBCR"
    if (reg == 0xF6):
        name = "P1CR"
    if (reg == 0xF8):
        name = "P1SR"
    return name
    
def translate(value):
    if (len(value) == 2):
        reg = int(value, 16)
        cmd = (reg * 4) & 0x3F0

        if (reg & 2):
            cmd |= 0x3000
        else:
            cmd |= 0x0C00

        cmd |= 0x4000;

        print hex(reg) + " -> " + hex(cmd) + " (" + get_name(reg) + ")"
    if (len(value) == 4):
        cmd = int(value, 16)
        reg = cmd & ~0x4000

        if (reg & 0x3000):
            reg &= ~0x3000
            reg |= 0x0008
        else:
            reg &= ~0x0C00

        reg = reg / 4;
        print hex(cmd) + " -> " + hex(reg) + " (" + get_name(reg) + ")"


argc = len(sys.argv)

if (argc < 2):
    print "Convert captured KSZ8851SNL SPI bytes to register addresses (and names)"
    print "and vise-versa"
    print "Example: ksz.py 4e40 7240"
    print "Example: ksz.py 90 92"
    
for x in range(1, argc):
    value = sys.argv[x]
    translate(value)
