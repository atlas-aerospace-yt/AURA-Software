/**
 * The code to run on core 1 of the ESP32-S3 processor.
 *
 */

#include "main_core_1.h"

void core_1_task(void* args) {
  uint32_t counter{};

  QueueHandle_t controlOutputQueue = (QueueHandle_t)args;
  sensor_data my_data;

  spi::create_spi_bus(SPI2_HOST, MOSI, MISO, SCLK);
  flash::flash my_flash(FLASH_CS, SPI2_HOST);
  sd_card::sd_card my_sd_card(SD_CARD_CS, SPI2_HOST);

  FILE* sd_file;

  // Clear file contents
  sd_file = fopen("/sd_card/data.txt", "w");
  fclose(sd_file);

  while (true) {
    if (xQueueReceive(controlOutputQueue, &my_data, MAX_DELAY)) {
      counter++;

      // Print data if connected to a computer
      printf("%f ", my_data.voltage);
      printf("%f ", my_data.current);
      printf("%f ", my_data.power);
      printf("%f ", my_data.ch1);
      printf("%f ", my_data.ch2);
      printf("%f ", my_data.ch3);
      printf("%f ", my_data.ch4);
      printf("%f ", my_data.ch5);
      printf("%f ", my_data.ch6);
      printf("Delta Time: %.8f\n", my_data.dt);

      graph("GyroX", my_data.gyro_x, TOP);
      graph("GyroY", my_data.gyro_y, TOP);
      graph("GyroZ", my_data.gyro_z, TOP);
      graph("SetX", my_data.set_x, TOP);
      graph("SetY", my_data.set_y, TOP);
      graph("SetZ", my_data.set_z, TOP);

      graph("Height", my_data.height, TOP);

      graph("PID_X", my_data.pid_x, BOT);
      graph("PID_Y", my_data.pid_y, BOT);
      graph("PID_Z", my_data.pid_z, BOT);
      graph("ESC_0", my_data.esc0, BOT);
      graph("ESC_1", my_data.esc1, BOT);
      graph("ESC_2", my_data.esc2, BOT);
      graph("ESC_3", my_data.esc3, BOT);

      // Write to flash (all in one fprintf for speed)
      sd_file = fopen("/sd_card/data.txt", "a");
      if (!sd_file) {
        ESP_LOGE("main", "Failed to open data.txt: errno=%d (%s)", errno,
                 strerror(errno));
        continue;
      }
      fprintf(sd_file,
              "%f, %f, %f, %f, %f, %f, "  // Transmitter
              "%f, %f, %f, %f, "          // ESC outputs
              "%f, %f, %f, "              // PID
              "%f, %f, %f, "              // Setpoints
              "%f, %f, %f, "              // Ori
              "%f, %f, %f, %f, "          // Gyro + dt
              "%f, %f, %f\n",             // Power
              my_data.ch1, my_data.ch2, my_data.ch3, my_data.ch4, my_data.ch5,
              my_data.ch6, my_data.esc0, my_data.esc1, my_data.esc2,
              my_data.esc3, my_data.pid_x, my_data.pid_y, my_data.pid_z,
              my_data.set_x, my_data.set_y, my_data.set_z, my_data.ori_x,
              my_data.ori_y, my_data.ori_z, my_data.gyro_x, my_data.gyro_y,
              my_data.gyro_z, my_data.dt, my_data.voltage, my_data.current,
              my_data.power);

      if (counter % 50 == 0) {
        fflush(sd_file);
        fsync(fileno(sd_file));
      }

      fclose(sd_file);
    }
  }
}
