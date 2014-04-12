# Dependencies
from PIL import Image
import argparse, os

# Compile an image into a bicolor pixels array
def compileImage(input_filename, fo):

    print 'Processing', input_filename

    # Open the input image file
    img = Image.open(input_filename)
    if not img:
        print "ERROR: Cannot open the input image file \"%s\"" % input_filename
        return 1

    # Compute the variable name and image size
    var_name = os.path.split(input_filename)[1].split('.')[0]
    var_name = ''.join(c for c in var_name if c.isalnum())
    width, height = img.size

    # Print the variable header
    fo.write("""const unsigned int img_%(varname)s_width = %(width)d;
const unsigned int img_%(varname)s_height = %(height)d;
""" % {'width': width, 'height': height, 'varname': var_name})

    # Turn the input image into RGB image
    rgb_img = img.convert('RGB')

    # Load pixels of image into memory
    px = rgb_img.load()

    # Process all lines
    for y in range(0, height):

        # Open the line array
        fo.write("const char img_%(varname)s_data_chunk%(lineIndex)d[] PROGMEM = \"" % {'varname': var_name, 'lineIndex' : y})

        # Process all pixels
        for x in range(0, width):
                
            # Get RGB values
            r, g, b = px[x, y]

            # Write to file
            fo.write("\\x%x\\x%x\\x%x" % (r, g, b))

        # Close the line array
        fo.write("\";\n")

    # Print the table variable header
    fo.write("const char* const img_%(varname)s_data[] PROGMEM = {\n" % {'varname': var_name})

    # Process all lines
    for y in range(0, height):

        # Write the chunk table entry
        fo.write("  img_%(varname)s_data_chunk%(lineIndex)d" % {'varname': var_name, 'lineIndex' : y})

        # Add the semicolon
        if y != (height - 1):
            fo.write(',\n')
        else:
            fo.write('\n')

    # Print the table variable footer
    fo.write('};\n\n')
    
    # No error
    return 0

def processFiles(filenames):
    # Process each images
    for filename in filenames:
        compileImage(filename, fo)

def processDirectories(directories):
    # Process each images
    for d in directories:
        for f in os.listdir(d):
            current_file = os.path.join(d, f)
            compileImage(current_file, fo)

# Create a new arguments parser
parser = argparse.ArgumentParser(description = 'Turn image(s) into C source file for the Skywodd RGB led matrix project')
parser.add_argument('input_filenames', metavar='INPUT FILE', nargs='+', help='Input image filename(s) to process')
parser.add_argument('output_filename', metavar='OUTPUT FILE', help='Output filename for the C file')
parser.add_argument('-d', '--directory', dest='process', action='store_const', const=processDirectories, default=processFiles, help='Process a whole directory instead of files.')

# Parse CLI arguments
args = parser.parse_args()

# Open the output file
fo = open(args.output_filename, 'w')
if not fo:
    print 'ERROR: Cannot open the output file'
    exit(1)

# Print file header
fo.write("""/* AUTO-GENERATED CODE DO NOT EDIT */

#include <avr/pgmspace.h>

""")

# Process input
args.process(args.input_filenames)

# Close the input and output file
fo.close()
