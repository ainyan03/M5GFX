// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#pragma once

// Included from M5GFX.cpp after the board-specific panel and light types are defined.
namespace board_detect
{
namespace m5
{
  struct display_parts_t
  {
    lgfx::Bus_SPI* bus = nullptr;
    lgfx::Panel_Device* panel = nullptr;
    lgfx::ILight* light = nullptr;
    lgfx::ITouch* touch = nullptr;
  };

  bool setup_esp32_d0wdq6(const board_result_t& result, display_parts_t* parts)
  {
    if (parts == nullptr || result.def == nullptr) { return false; }

    auto bus = new lgfx::Bus_SPI();
    auto bus_cfg = bus->config();
    bus_cfg.freq_write = 8000000;
    bus_cfg.freq_read = 8000000;
    bus_cfg.spi_mode = 0;
    bus_cfg.spi_3wire = true;
    bus_cfg.use_lock = true;
    bus_cfg.spi_host = SPI3_HOST;
    bus_cfg.dma_channel = 1;

    const auto board = static_cast<lgfx::board_t>(result.def->id);
    switch (board)
    {
    case lgfx::board_M5Station:
      bus_cfg.spi_host = SPI2_HOST;
      bus_cfg.freq_write = 40000000;
      bus_cfg.freq_read = 15000000;
      bus_cfg.pin_mosi = GPIO_NUM_23;
      bus_cfg.pin_miso = -1;
      bus_cfg.pin_sclk = GPIO_NUM_18;
      bus_cfg.pin_dc = GPIO_NUM_19;
      bus->config(bus_cfg);
      {
        auto panel = new Panel_M5StickCPlus();
        auto cfg = panel->config();
        cfg.pin_rst = GPIO_NUM_15;
        cfg.offset_rotation = 1;
        panel->config(cfg);
        panel->setRotation(0);
        panel->bus(bus);
        parts->panel = panel;
      }
      parts->light = new Light_M5Tough();
      break;

    case lgfx::board_M5StackCore2:
    case lgfx::board_M5Tough:
      // SPI3 is shared with the SD slot on this family; changing hosts would
      // make the display work while silently breaking SD access.
      bus_cfg.freq_write = 40000000;
      bus_cfg.freq_read = 16000000;
      bus_cfg.pin_mosi = GPIO_NUM_23;
      bus_cfg.pin_miso = GPIO_NUM_38;
      bus_cfg.pin_sclk = GPIO_NUM_18;
      bus_cfg.pin_dc = GPIO_NUM_15;
      bus->config(bus_cfg);
      {
        lgfx::Panel_ILI9342* panel;
        if (result.option & option_lcd_e)
        {
          panel = new Panel_M5StackCore2E();
          // The E dummy-pixel setting depends on the final 16 MHz read clock,
          // so apply it only after that bus configuration has been selected.
          _set_ili9342e_read(panel, bus_cfg.freq_read);
        }
        else
        {
          panel = new Panel_M5StackCore2();
        }
        panel->bus(bus);
        parts->panel = panel;
      }

      if (board == lgfx::board_M5StackCore2)
      {
        parts->light = (result.option & option_core2_new_pmic)
                     ? static_cast<lgfx::ILight*>(new Light_M5StackCore2_AXP2101())
                     : static_cast<lgfx::ILight*>(new Light_M5StackCore2());
        auto touch = new lgfx::Touch_FT5x06();
        auto cfg = touch->config();
        cfg.pin_int = GPIO_NUM_39;
        cfg.pin_sda = GPIO_NUM_21;
        cfg.pin_scl = GPIO_NUM_22;
        cfg.i2c_addr = 0x38;
        cfg.i2c_port = I2C_NUM_1;
        cfg.freq = 400000;
        cfg.x_min = 0;
        cfg.x_max = 319;
        cfg.y_min = 0;
        cfg.y_max = 279;
        cfg.bus_shared = false;
        touch->config(cfg);
        parts->panel->touch(touch);
        float affine[6] = { 1, 0, 0, 0, 1, 0 };
        parts->panel->setCalibrateAffine(affine);
        parts->touch = touch;
      }
      else
      {
        parts->light = new Light_M5Tough();
        auto touch = new lgfx::Touch_CHSC6540();
        auto cfg = touch->config();
        cfg.pin_int = GPIO_NUM_39;
        cfg.pin_sda = GPIO_NUM_21;
        cfg.pin_scl = GPIO_NUM_22;
        cfg.i2c_addr = 0x2E;
        cfg.i2c_port = I2C_NUM_1;
        cfg.freq = 400000;
        cfg.x_min = 0;
        cfg.x_max = 319;
        cfg.y_min = 0;
        cfg.y_max = 239;
        cfg.bus_shared = false;
        touch->config(cfg);
        parts->panel->touch(touch);
        parts->touch = touch;
      }
      break;

    case lgfx::board_M5Stack:
      bus_cfg.freq_write = 40000000;
      bus_cfg.freq_read = 16000000;
      bus_cfg.pin_mosi = GPIO_NUM_23;
      bus_cfg.pin_miso = GPIO_NUM_19;
      bus_cfg.pin_sclk = GPIO_NUM_18;
      bus_cfg.pin_dc = GPIO_NUM_27;
      bus->config(bus_cfg);
      {
        auto panel = new Panel_M5Stack();
        panel->bus(bus);
        parts->panel = panel;
      }
      {
        auto light = new lgfx::Light_PWM();
        auto cfg = light->config();
        cfg.pin_bl = GPIO_NUM_32;
        cfg.freq = 44100;
        cfg.pwm_channel = 7;
        light->config(cfg);
        parts->light = light;
      }
      break;

    case lgfx::board_M5Paper:
      bus_cfg.freq_write = 40000000;
      bus_cfg.freq_read = 20000000;
      bus_cfg.pin_mosi = GPIO_NUM_12;
      bus_cfg.pin_miso = GPIO_NUM_13;
      bus_cfg.pin_sclk = GPIO_NUM_14;
      bus_cfg.pin_dc = -1;
      bus_cfg.spi_3wire = false;
      bus->config(bus_cfg);
      {
        auto panel = new lgfx::Panel_IT8951();
        auto cfg = panel->config();
        cfg.panel_height = 540;
        cfg.panel_width = 960;
        cfg.pin_cs = GPIO_NUM_15;
        cfg.pin_rst = GPIO_NUM_23;
        cfg.pin_busy = GPIO_NUM_27;
        cfg.offset_rotation = 3;
        panel->config(cfg);
        panel->bus(bus);
        parts->panel = panel;
      }
      {
        auto touch = new lgfx::Touch_GT911();
        auto cfg = touch->config();
        cfg.pin_int = GPIO_NUM_36;
        cfg.pin_sda = GPIO_NUM_21;
        cfg.pin_scl = GPIO_NUM_22;
#ifdef _M5EPD_H_
        cfg.i2c_port = I2C_NUM_0;
#else
        cfg.i2c_port = I2C_NUM_1;
#endif
        cfg.freq = 400000;
        cfg.x_min = 0;
        cfg.x_max = 539;
        cfg.y_min = 0;
        cfg.y_max = 959;
        cfg.offset_rotation = 1;
        cfg.bus_shared = false;
        touch->config(cfg);
        parts->panel->touch(touch);
        parts->touch = touch;
      }
      break;

    default:
      delete bus;
      return false;
    }

    parts->bus = bus;
    return true;
  }
}
}
