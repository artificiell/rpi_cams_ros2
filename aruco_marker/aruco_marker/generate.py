#!/usr/bin/env python3
"""
Script for generating Aruco marker images.
"""

import argparse
import cv2
import numpy as np
from PIL import Image

class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter):
    """ Trick to allow both defaults and nice formatting in the help. """
    pass

# Main function
def main():
    parser = argparse.ArgumentParser(formatter_class=CustomFormatter,
                                     description="Generate a .png image of a specified maker.")
    parser.add_argument('--id', default=1, type=int,
                        help = 'Marker id to generate')
    parser.add_argument('--size', default=0.1, type=float,
                        help = 'Marker side length in meters')
    parser.add_argument('--dpi', default=300, type=int,
                        help = 'Dots per inch for rendering the marker')
    
    dict_options = [s for s in dir(cv2.aruco) if s.startswith("DICT")]
    option_str = ", ".join(dict_options)
    dict_help = "Dictionary to use. Valid options include: {}".format(option_str)
    parser.add_argument('--dictionary', default="DICT_ARUCO_ORIGINAL", type=str,
                        choices=dict_options,
                        help=dict_help, metavar='')
    args = parser.parse_args()

    # Convert meters to pixels
    inches_per_meter = 39.3701
    pixel_size = int(round(args.size * inches_per_meter * args.dpi))

    # Generate the marker
    dictionary_id = cv2.aruco.__getattribute__(args.dictionary)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    image = np.zeros((pixel_size, pixel_size), dtype=np.uint8)
    image = cv2.aruco.generateImageMarker(dictionary, args.id, pixel_size, image, 1)

    # Convert to PIL image and save with (right) DPI
    pil_image = Image.fromarray(image)
    filename = f"marker_{args.id:04d}.png"
    pil_image.save(filename, dpi=(args.dpi, args.dpi))

if __name__ == "__main__":
    main()
