import tensorflow as tf
import numpy as np
import hexdump

# Load your TFLite model
tflite_model_path =  'C:\\Users\\PC-kun\\Desktop\\MoGlove ML\\MoGloveModel.tflite'
with open(tflite_model_path, 'rb') as file:
    tflite_model = file.read()

hex_lines = []
for chunk in hexdump.chunks(tflite_model, 16):
    hex_lines.append('0x' + hexdump.dump(chunk, sep=', 0x'))

# Save the hex dump in model.h5
with open('model.h5', 'w') as f:
    f.write('const unsigned char modelBin[] = {\n')
    for index, line in enumerate(hex_lines):
        f.write('  ' + line)
        if index < len(hex_lines) - 1:
            f.write(',')
        f.write('\n')
    f.write('};\n\n')

    f.write('const unsigned int modelLen = {};\n'.format(len(tflite_model)))

    f.write('const byte charMap[] = {\n')
    highest_key = np.max(list(char_map.keys()))
    for i in range(highest_key + 1):
        if i % 10 == 0:
            f.write('  ')

        if i in char_map:
            f.write(hex(ord(char_map[i])))
        else:
            f.write('0x00')

        if i < highest_key:
            f.write(', ')
            if i % 10 == 9:
                f.write('\n')
    f.write('\n};\n\n')

    f.write('#define charCount {}\n'.format(len(char_map)))
