import requests
from PIL import Image
import numpy as np
import sys
import os
import math
import shutil


def convert_png_bmp(inputfilename, outputfilename):
    def convert_to_rgb565(image):
        """Convert an image to 16-bit RGB565 format, with transparent pixels set to white."""
        # Convert to RGBA to access alpha channel
        image = image.convert('RGBA')
        np_image = np.array(image)
        
        # Extract RGB and alpha channels
        r = np_image[:, :, 0]
        g = np_image[:, :, 1]
        b = np_image[:, :, 2]
        alpha = np_image[:, :, 3]
        
        # Convert RGB888 to RGB565
        r = (r >> 3).astype(np.uint16)
        g = (g >> 2).astype(np.uint16)
        b = (b >> 3).astype(np.uint16)
        
        # Create RGB565 pixel values
        rgb565 = (r << 11) | (g << 5) | b
        
        # Set transparent pixels (alpha = 0) to white (0xFFFF in RGB565)
        rgb565[alpha == 0] = 0xFFFF
        
        return rgb565

    def save_bmp_rgb565(image, output_filename):
        # Convert image to RGB565 format and flip it vertically
        rgb565_data = convert_to_rgb565(image.transpose(Image.FLIP_TOP_BOTTOM))
        
        height, width = rgb565_data.shape
        padded_row_size = (width * 2 + 3) & ~3  # Ensure row size is a multiple of 4 bytes
        padding = padded_row_size - (width * 2)

        pixel_data = bytearray()
        for row in rgb565_data:
            pixel_data.extend(row.tobytes())
            pixel_data.extend(b'\x00' * padding)  # Add padding to each row

        file_header = bytearray([
            0x42, 0x4D,                             # 'BM' header
            0, 0, 0, 0,                             # File size (will be filled later)
            0, 0, 0, 0,                             # Reserved
            54 + 12, 0, 0, 0                        # Offset to pixel data (54 bytes for header + 12 bytes for masks)
        ])

        info_header = bytearray([
            40, 0, 0, 0,            # Header size (40 bytes)
            width & 0xFF, (width >> 8) & 0xFF, (width >> 16) & 0xFF, (width >> 24) & 0xFF,  # Image width (4 bytes)
            height & 0xFF, (height >> 8) & 0xFF, (height >> 16) & 0xFF, (height >> 24) & 0xFF,  # Image height (4 bytes)
            1, 0,                   # Planes (2 bytes)
            16, 0,                  # Bits per pixel (16 bits = 2 bytes per pixel)
            3, 0, 0, 0,             # Compression (3 = BI_BITFIELDS, to specify bit masks)
            len(pixel_data) & 0xFF, (len(pixel_data) >> 8) & 0xFF, (len(pixel_data) >> 16) & 0xFF, (len(pixel_data) >> 24) & 0xFF,  # Image size (4 bytes)
            0, 0, 0, 0,             # X pixels per meter (unused, 4 bytes)
            0, 0, 0, 0,             # Y pixels per meter (unused, 4 bytes)
            0, 0, 0, 0,             # Colors used (0 for no palette)
            0, 0, 0, 0              # Important colors (0 for no palette)
        ])

        # Bit masks for RGB565 format (5 bits for Red, 6 bits for Green, 5 bits for Blue)
        red_mask = bytearray([0x00, 0xF8, 0x00, 0x00])
        green_mask = bytearray([0xE0, 0x07, 0x00, 0x00])
        blue_mask = bytearray([0x1F, 0x00, 0x00, 0x00])

        # Fill in file size
        file_size = len(file_header) + len(info_header) + len(red_mask) + len(green_mask) + len(blue_mask) + len(pixel_data)
        file_header[2:6] = [
            file_size & 0xFF, (file_size >> 8) & 0xFF,
            (file_size >> 16) & 0xFF, (file_size >> 24) & 0xFF
        ]

        # Write BMP file
        with open(output_filename, 'wb') as f:
            f.write(file_header)
            f.write(info_header)
            f.write(red_mask)
            f.write(green_mask)
            f.write(blue_mask)
            f.write(pixel_data)

    # Example usage
    image = Image.open(inputfilename)
    save_bmp_rgb565(image, outputfilename)


infile = input("Enter input filename:")
outfile = input("Enter output filename:")
convert_png_bmp(infile, outfile)