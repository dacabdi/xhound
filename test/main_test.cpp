#include <Arduino.h>
#include <unity.h>

#include "eavesdrop.h"
#include "mock_stream.h"
#include "mock_gps.h"

// void setUp(void) {
// // set stuff up here
// }

// void tearDown(void) {
// // clean stuff up here
// }

using namespace GNSS_RTK_ROVER;

void test_should_passthrough_all_available_data(void)
{
    uint8_t input_buffer[4] = {0x01, 0x02, 0x03, 0x04}; 
    uint8_t output_buffer[4] = {0x00};
    MockStream input(input_buffer, 4);
    MockStream output(output_buffer, 4);
    SimpleEavesdropper eavesdropper(input, output);

    eavesdropper.eavesdrop();

    TEST_ASSERT_EQUAL_CHAR_ARRAY(input_buffer, output_buffer, 4);
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();
}

void loop() 
{    
    RUN_TEST(test_should_passthrough_all_available_data);
    UNITY_END(); // stop unit testing
}