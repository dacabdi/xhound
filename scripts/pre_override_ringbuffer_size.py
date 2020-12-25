""" override in source the api/RingBuffer.h SERIAL_BUFFER_SIZE macro for RingBuffer type """
import re
from common import replaceAll

Import("env")

projectsDirPath = env.Dictionary()['PROJECT_PACKAGES_DIR']
filePath = projectsDirPath + "/framework-arduino-samd/cores/arduino/api/RingBuffer.h"

# TODO use regex for any original buffer size, hint -> re.compile(r'#define SERIAL_BUFFER_SIZE \d+$')
replaceAll(filePath, "SERIAL_BUFFER_SIZE 64", "SERIAL_BUFFER_SIZE 1024")