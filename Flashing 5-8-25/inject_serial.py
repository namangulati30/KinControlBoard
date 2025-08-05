from intelhex import IntelHex

def inject_serial_number(input_hex, output_hex, serial_number, address=0x0803F800):
    """
    Inject an 18-byte serial number into a hex file at the specified address.
    
    Args:
        input_hex (str): Path to the input hex file.
        output_hex (str): Path to the output hex file.
        serial_number (str): Serial number to inject (up to 18 characters).
        address (int): Flash address to write the serial number (default: 0x0803F800).
    """
    # Initialize IntelHex object with input file
    ih = IntelHex(input_hex)
    
    # Convert serial number to bytes (ASCII) and ensure it fits in 18 bytes
    serial_bytes = serial_number.encode('ascii')[:18]
    serial_bytes = serial_bytes + b'\x00' * (18 - len(serial_bytes))  # Pad with zeros
    
    # Write serial number to the specified address
    for i, byte in enumerate(serial_bytes):
        ih[address + i] = byte
    
    # Save the modified hex file
    ih.write_hex_file(output_hex)
    print(f"Generated {output_hex} with serial number '{serial_number}' at address 0x{address:08X}")

# Example usage
if __name__ == "__main__":
    input_hex = "firmware.hex"  # Your Keil-generated hex file
    serial_number = "SN1234567891234567"    # Example serial number
    output_hex = f"firmware_{serial_number}.hex"  # Output file
    inject_serial_number(input_hex, output_hex, serial_number)