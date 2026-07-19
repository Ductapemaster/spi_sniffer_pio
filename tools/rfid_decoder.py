#!/usr/bin/env python3
import re
import sys
import argparse

# ==============================================================================
# FILTER SETTINGS
# ==============================================================================
# True:  Hides relentless Status2Reg read polling loops when idle.
# False: Displays absolutely every single transaction captured on the bus.
HURRY_MODE = True  

# ==============================================================================
# MFRC522 / NZ3801 DATASHEET REGISTERS MAP (Pages 0 to 3)
# ==============================================================================
REGISTERS = {
    # Page 0: Command and Status
    0x01: "CommandReg",    0x02: "ComIEnReg",     0x03: "DivIEnReg",
    0x04: "ComIrqReg",     0x05: "DivIrqReg",     0x06: "ErrorReg",
    0x07: "Status1Reg",    0x08: "Status2Reg",    0x09: "FIFODataReg",
    0x0A: "FIFOLevelReg",  0x0B: "WaterLevelReg", 0x0C: "ControlReg",
    0x0D: "BitFramingReg", 0x0E: "CollReg",
    # Page 1: Command Configuration
    0x11: "ModeReg",       0x12: "TxModeReg",     0x13: "RxModeReg",
    0x14: "TxControlReg",  0x15: "TxASKReg",      0x16: "TxSelReg",
    0x17: "RxSelReg",      0x18: "RxThresholdReg",0x19: "DemodReg",
    0x1C: "MfTxReg",       0x1D: "MfRxReg",       0x1F: "SerialSpeedReg",
    # Page 2: Configuration
    0x21: "CRCResultRegH", 0x22: "CRCResultRegL", 0x24: "ModWidthReg",
    0x26: "RFCfgReg",      0x27: "GsNReg",        0x28: "CWGsPReg",
    0x29: "ModGsPReg",     0x2A: "TModeReg",      0x2B: "TPrescalerReg",
    0x2C: "TReloadRegH",   0x2D: "TReloadRegL",   0x2E: "TCounterValRegH",
    0x2F: "TCounterValRegL",
    # Page 3: Test Registers
    0x31: "TestSel1Reg",   0x32: "TestSel2Reg",   0x33: "TestPinReg",
    0x34: "TestPinValueReg",0x35: "TestBusReg",   0x36: "AutoTestReg",
    0x37: "VersionReg",    0x38: "AnalogTestReg", 0x39: "TestDAC1Reg",
    0x3A: "TestDAC2Reg",   0x3B: "TestADCReg"
}

COMMANDS = {
    0x00: "Idle",           0x01: "Mem",           0x02: "GenerateIDs",
    0x03: "CalcCRC",        0x04: "Transmit",      0x08: "NoCmdChange",
    0x09: "Receive",        0x0C: "Transceive",    0x0F: "SoftReset",
}

def log_to_file(text, filename):
    """Strips ANSI color escape codes and appends text to the target file"""
    clean_text = re.sub(r'\x1b\[[0-9;]*m', '', text)
    with open(filename, "a", encoding="utf-8") as f:
        f.write(clean_text + "\n")
        
def parse_bitfields(reg_name, val, is_read):
    """Parses raw byte values into literal datasheet register configurations"""
    if is_read:
        if reg_name == "CommandReg":
            return f"Status -> Executing/Idle (Raw status: 0x{val:02X})"
        return f"Returned Hex: {val:02X}"
        
    if reg_name == "TxControlReg":
        drivers = []
        if val & 0x01: drivers.append("TX1 Antenna Driver Enabled")
        if val & 0x02: drivers.append("TX2 Antenna Driver Enabled")
        if val & 0x80: drivers.append("InvTx2RFOn (Differential Phase Modulation)")
        return "Antenna Output Configuration: " + (", ".join(drivers) if drivers else "All Drivers Disabled")
    elif reg_name == "ModeReg":
        crc_presets = ["0000h", "6363h (ISO14443-A Standard)", "A671h", "FFFFh"]
        preset = crc_presets[(val >> 4) & 0x03]
        return f"Mode Settings: CRC Preset set to {preset} | TxWaitRF Enabled"
    elif reg_name == "ControlReg":
        modes = []
        if val & 0x10: modes.append("Initiator Mode (Reader Mode active)")
        if val & 0x08: modes.append("Crypto1On (Mifare Hardware Crypto Unit Enabled)")
        return "Control Configuration: " + (", ".join(modes) if modes else f"Value: 0x{val:02X}")
    elif reg_name in ["TxModeReg", "RxModeReg"]:
        speed_code = (val >> 4) & 0x07
        baud_rate = 106 * (2 ** speed_code)
        crc_status = "Enabled" if val & 0x80 else "Disabled"
        direction = "TX" if reg_name == "TxModeReg" else "RX"
        return f"{direction} Framing Config: Type A, {baud_rate} kbd | Hardware CRC: {crc_status}"
    elif reg_name == "RFCfgReg":
        gains = ["23 dB", "33 dB", "38 dB", "43 dB", "48 dB", "54 dB", "59 dB", "23 dB"]
        gain_idx = (val >> 4) & 0x07
        return f"Internal Receiver Amplifier Gain tuned to: {gains[gain_idx]} (Raw: 0x{val:02X})"
    elif reg_name == "GsNReg":
        return f"Antenna Tuning -> NMOS Conductance (GsN) level: 0x{val:02X}"
    elif reg_name == "CWGsPReg":
        return f"Antenna Tuning -> PMOS Conductance (CWGsP) during unmodulated periods: 0x{val:02X}"
    elif reg_name == "ModWidthReg":
        return f"Modulation Pulse Width configuration index: 0x{val:02X}"
        
    return f"Value Written: 0x{val:02X}"

def decode_iso14443_cmd(fifo_bytes):
    """Decodes buffered FIFO arrays into high-level ISO14443 contactless primitives"""
    if not fifo_bytes:
        return "Empty FIFO transmission"
    
    prime_byte = fifo_bytes[0]
    if prime_byte == 0x26: return "REQA (Request Command - Type A)"
    if prime_byte == 0x52: return "WUPA (Wake-Up Command - Type A)"
    if prime_byte == 0x93:
        if len(fifo_bytes) > 1 and fifo_bytes[1] == 0x20:
            return "ISO14443-3 -> [Anticollision Cascade 1 Selection]"
        if len(fifo_bytes) > 1 and fifo_bytes[1] == 0x70:
            uid_str = ''.join(f'{b:02X}' for b in fifo_bytes[2:6])
            return f"ISO14443-3 -> [SELECT Cascade 1] targeting UID: {uid_str}"
        return "Mifare Base Command 0x93"
    if prime_byte == 0x95: return "ISO14443-3 -> [Anticollision Cascade 2 Selection]"
    
    return f"Custom FIFO Data Stream: {' '.join(f'{b:02X}' for b in fifo_bytes)}"

def process_single_frame(frame_content, state):
    """Parses an isolated frame block extraction sequence"""
    if state.get('compact'):
        pattern = r'([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})'
    else:
        pattern = r'\[([0-9A-Fa-f]{2})-([0-9A-Fa-f]{2})\]'
        
    blocks = re.findall(pattern, frame_content)
    if not blocks:
        return
        
    cmd_mosi, _ = int(blocks[0][0], 16), int(blocks[0][1], 16)
    is_read = bool(cmd_mosi & 0x80)
    reg_addr = (cmd_mosi >> 1) & 0x3F
    reg_name = REGISTERS.get(reg_addr, f"UnknownReg_0x{reg_addr:02X}")
    
    data_bytes = []
    for mosi_str, miso_str in blocks[1:]:
        b_mosi = int(mosi_str, 16)
        b_miso = int(miso_str, 16)
        data_bytes.append(b_miso if is_read else b_mosi)

    # ==============================================================================
    # EXTENDED HURRY_MODE FILTER (Filtro de ruido masivo en reposo)
    # ==============================================================================
    if HURRY_MODE and is_read:
        if reg_name == "Status2Reg":
            return
        if reg_name == "ComIrqReg" and data_bytes and data_bytes[0] == 0x44:
            return
            
    state['frame_count'] += 1
    direction_tag = "[READ]" if is_read else "[WRITE]"
    op_str = f"{direction_tag} {reg_name}"
    
    desc = ""
    if not is_read:
        if reg_name == "FIFODataReg":
            state['virtual_fifo'].extend(data_bytes)
            desc = f"Pushing into FIFO -> Bytes: {' '.join(f'{b:02X}' for b in data_bytes)}"
        elif reg_name == "FIFOLevelReg" and any(b & 0x80 for b in data_bytes):
            state['virtual_fifo'].clear()
            desc = "FlushBuffer -> Cleared Virtual Shadow FIFO"
        elif reg_name == "CommandReg" and data_bytes:
            cmd_val = data_bytes[0] & 0x0F
            cmd_name = COMMANDS.get(cmd_val, f"CustomCmd_0x{cmd_val:02X}")
            if cmd_name == "Transceive":
                context = decode_iso14443_cmd(state['virtual_fifo'])
                desc = f"Executing \033[93mTRANSCEIVE (0x0C)\033[0m | {context}"
            else:
                desc = f"Command Issued -> {cmd_name} Instruction"
        else:
            desc = parse_bitfields(reg_name, data_bytes[0] if data_bytes else 0, False)
    else:
        if reg_name == "FIFOLevelReg" and data_bytes:
            desc = f"FIFO status checked -> {data_bytes[0]} byte(s) pending extraction"
        elif reg_name == "FIFODataReg" and data_bytes:
            state['accumulated_reads'].extend(data_bytes)
            desc = f"Popping from FIFO -> Bytes: {' '.join(f'{b:02X}' for b in data_bytes)}"
            
            if len(state['accumulated_reads']) == 5:
                b1, b2, b3, b4, bcc = state['accumulated_reads']
                calc_bcc = b1 ^ b2 ^ b3 ^ b4
                status = "\033[92m[BCC OK]\033[0m" if calc_bcc == bcc else "\033[91m[BCC ERROR]\033[0m"
                desc += f"   >>> \033[96m[CARD CAPTURED]\033[0m UID: {b1:02X}{b2:02X}{b3:02X}{b4:02X} | Verification: {status} <<<"
                state['accumulated_reads'].clear()
        else:
            desc = parse_bitfields(reg_name, data_bytes[0] if data_bytes else 0, True)
            
    if reg_name not in ["FIFODataReg", "FIFOLevelReg"]:
        state['accumulated_reads'].clear()
      
    lines = desc.split('\n')
    main_line = f"[{state['frame_count']:04d}] | {op_str:<35} | {lines[0]}"
    print(main_line)
    
    if state.get('output_file'):
        log_to_file(main_line, state['output_file'])
        
    for extra_line in lines[1:]:
        sub_line = f"{' ' * 8}| {' ' * 35} | {extra_line}"
        print(sub_line)
        if state.get('output_file'):
            log_to_file(sub_line, state['output_file'])    

def parse_stream(stream_generator, output_file=None, compact=False):
    """Accumulates incoming data chunks and extracts sequential frame tokens safely"""
    state = {
        'virtual_fifo': [], 
        'accumulated_reads': [], 
        'frame_count': 0,
        'output_file': output_file,
        'compact': compact
    }
    
    header = f"{'FRAME':<7} | {'SPI BUS OPERATION':<35} | INTERPRETATION / DATASHEET BITFIELD ANALYSIS"
    divider = "-" * 120
    
    print(header)
    print(divider)

    if output_file:
        log_to_file(header, output_file)
        log_to_file(divider, output_file)
        
    accumulator = ""
    for chunk in stream_generator:
        accumulator += chunk
        while True:
            s_idx = accumulator.find('S')
            if s_idx == -1:
                if len(accumulator) > 2048:
                    accumulator = accumulator[-256:]
                break
                
            p_idx = accumulator.find('P', s_idx)
            if p_idx == -1:
                accumulator = accumulator[s_idx:]
                break
                
            frame_content = accumulator[s_idx+1:p_idx]
            process_single_frame(frame_content, state)
            accumulator = accumulator[p_idx+1:]           

def file_chunk_generator(filepath):
    """Streams data chunks safely out of local text files"""
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            while True:
                chunk = f.read(1024)
                if not chunk:
                    break
                yield chunk
    except FileNotFoundError:
        print(f"Error: Target log file not found at path: {filepath}")
        sys.exit(1)

def serial_chunk_generator(port, baud):
    """Streams live captured text out of a physical USB CDC interface device"""
    try:
        import serial
    except ImportError:
        print("Error: 'pyserial' framework is missing.")
        print("Install it into your system using: pip install pyserial")
        sys.exit(1)
        
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            ser.reset_input_buffer()
            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    yield data.decode('utf-8', errors='ignore')
    except serial.SerialException as e:
        print(f"Serial Port Error structural breakdown on device {port}: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nLive serial interface trace terminated by user.")
        sys.exit(0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stateful MFRC522/NZ3801 SPI Sniffer Stream Parser")
    input_group = parser.add_mutually_exclusive_group(required=True)
    
    input_group.add_argument("-f", "--file", help="Path to a saved sniffer log dump file")
    input_group.add_argument("-s", "--serial", help="System identifier for local live serial port connection (e.g. /dev/ttyACM0)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Live serial transmission connection baud rate speed (Default: 115200)")
    parser.add_argument("-o", "--output", help="Path to save the clean, color-free log file (optional)")
    parser.add_argument("-c", "--compact", action="store_true", help="Decode the new ultra-compact format (S88000044P) instead of extended format")
    
    args = parser.parse_args()
    
    if args.output:
        try:
            with open(args.output, 'w', encoding='utf-8') as f:
                f.write("") 
        except Exception as e:
            print(f"Error initializing output file: {e}")
            sys.exit(1)
    
    if args.file:
        stream = file_chunk_generator(args.file)
    else:
        print(f"Opening live port monitoring hook directly onto: {args.serial} at {args.baud} baud...")
        stream = serial_chunk_generator(args.serial, args.baud)

    parse_stream(stream, output_file=args.output, compact=args.compact)
