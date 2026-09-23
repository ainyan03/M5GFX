// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#pragma once

#include "../board_detect.hpp"

#include <new>

namespace m5gfx
{
namespace board_detect
{
namespace m5
{
  namespace
  {
    constexpr board_id_t id(lgfx::board_t value)
    {
      return static_cast<board_id_t>(value);
    }
  }

  // option bit 0: the power-management IC is the newer variant (0 = original variant)
  // option bit 1: the LCD controller is the E variant (0 = C variant)
  static const board_def_t board_core2  = { id(lgfx::board_M5StackCore2), "M5StackCore2", 0 };
  // option bit 1: the LCD controller is the E variant (0 = C variant)
  // Tough currently ships with AXP192 only, so option bit 0 is intentionally unused.
  static const board_def_t board_tough  = { id(lgfx::board_M5Tough), "M5Tough", 0 };
  // No configuration variants are currently distinguished.
  static const board_def_t board_station = { id(lgfx::board_M5Station), "M5Station", 0 };
  // option bit 0: IPS panel (0 = TN panel), sampled by the LCD reset strap
  // Pull signature measured on the earliest M5Stack BASIC (12 kOhm) and BASIC v2.61.
  // Core2 v1.1 sees the same SD wires as floating because their pull-ups are on
  // a PMIC-switched rail, so the fixed-pull signature below is Stack-only.
  static const board_def_t board_stack   = { id(lgfx::board_M5Stack), "M5Stack", 0 };
  // No configuration variants are currently distinguished.
  static const board_def_t board_paper   = { id(lgfx::board_M5Paper), "M5Paper", 0 };

  static constexpr std::uint32_t option_core2_new_pmic = 1u << 0;
  static constexpr std::uint32_t option_lcd_e = 1u << 1;
  static constexpr std::uint32_t option_stack_ips = 1u << 0;

  namespace detail
  {
    static constexpr std::uint8_t pmic_addr = 0x34;
    static constexpr std::uint32_t i2c_freq = 400000;
    static constexpr std::uint64_t sd_pull_mask = (std::uint64_t(1) << GPIO_NUM_4)
                                                 | (std::uint64_t(1) << GPIO_NUM_18)
                                                 | (std::uint64_t(1) << GPIO_NUM_23);

    struct i2c_scope_t
    {
      i2c_scope_t(probe_ctx_t& ctx, int sda, int scl)
      : i2c_scope_t(ctx.i2c_port_probe, sda, scl) {}

      i2c_scope_t(int port_, int sda, int scl)
      : port(port_), pins { sda, scl }
      {
        opened = lgfx::i2c::init(port, sda, scl).has_value();
      }

      ~i2c_scope_t()
      {
        if (opened) { lgfx::i2c::release(port); }
        for (auto& pin : pins) { pin.restore(); }
      }

      int port;
      lgfx::gpio::pin_backup_t pins[2];
      bool opened = false;
    };

    void pin_level(int pin, bool high)
    {
      if (high) { lgfx::gpio_hi(pin); }
      else      { lgfx::gpio_lo(pin); }
      lgfx::pinMode(pin, lgfx::pin_mode_t::output);
    }

    class retained_pin_guard_t
    {
    public:
      explicit retained_pin_guard_t(int pin) : pin_(pin), backup_(pin) {}
      ~retained_pin_guard_t()
      {
        if (active_ && !retained_) { backup_.restore(); }
      }

      void activate_high()
      {
        pin_level(pin_, true);
        active_ = true;
      }
      void retain() { retained_ = true; }

    private:
      int pin_;
      lgfx::gpio::pin_backup_t backup_;
      bool active_ = false;
      bool retained_ = false;
    };

    void pin_reset(int pin, bool reset)
    {
      lgfx::gpio_hi(pin);
      lgfx::pinMode(pin, lgfx::pin_mode_t::output);
      lgfx::delay(1);
      if (!reset) { return; }
      lgfx::gpio_lo(pin);
      lgfx::delay(2);
      lgfx::gpio_hi(pin);
      lgfx::delay(10);
    }

    std::uint32_t read_panel_id(soft_spi_t& bus, int pin_cs,
                                std::uint8_t cmd = 0x04, std::uint8_t dummy_bits = 1)
    {
      bus.beginTransaction();
      pin_level(pin_cs, true);
      bus.writeCommand(0, 8);
      bus.wait();
      lgfx::gpio_lo(pin_cs);
      bus.writeCommand(cmd, 8);
      bus.beginRead(dummy_bits);
      const auto value = bus.readData(32);
      lgfx::gpio_hi(pin_cs);
      bus.endTransaction();
      ESP_LOGD("board_detect_m5", "read cmd:%02x = %08x",
               static_cast<unsigned>(cmd), static_cast<unsigned>(value));
      return value;
    }

    void set_sd_spi_mode_soft(int pin_sclk, int pin_mosi, int pin_miso, int pin_cs)
    {
      lgfx::gpio::pin_backup_t pins[] = { pin_sclk, pin_mosi, pin_miso };

      soft_spi_t bus(pin_sclk, pin_mosi, pin_miso, -1, 2);
      bus.init();
      bus.beginTransaction();
      pin_level(pin_cs, true);

      // A native-mode card watches CMD regardless of CS. Make the mode change
      // the first shared-wire operation. CMD58 distinguishes a card already in
      // SPI mode, which must not be reset to idle by another CMD0.
      for (int i = 0; i < 10; ++i) { bus.writeData(0xFF, 8); }
      lgfx::gpio_lo(pin_cs);
      static constexpr std::uint8_t cmd58[] = { 0x7A, 0, 0, 0, 0, 0xFD, 0xFF, 0xFF };
      std::uint8_t response[sizeof(cmd58)];
      for (std::size_t i = 0; i < sizeof(cmd58); ++i)
      {
        response[i] = static_cast<std::uint8_t>(bus.transferData(cmd58[i], 8));
      }
      if (response[6] == response[7])
      {
        lgfx::gpio_hi(pin_cs);
        for (int i = 0; i < 10; ++i) { bus.writeData(0xFF, 8); }
        lgfx::gpio_lo(pin_cs);
        static constexpr std::uint8_t cmd0[] = { 0x40, 0, 0, 0, 0, 0x95, 0xFF, 0xFF };
        for (auto value : cmd0) { bus.transferData(value, 8); }
      }
      lgfx::gpio_hi(pin_cs);
      bus.endTransaction();

      // Signal pins are restored; the caller owns CS so it can keep it
      // inactive after a match. The card's SPI mode is intentionally retained.
      for (auto& pin : pins) { pin.restore(); }
    }

    bool write_masked(int port, std::uint8_t reg, std::uint8_t data, std::uint8_t mask)
    {
      return lgfx::i2c::writeRegister8(port, pmic_addr, reg, data, mask, i2c_freq).has_value();
    }

    bool write_sequence(int port, const std::uint8_t* sequence)
    {
      for (; sequence[0] != 0xFF || sequence[1] != 0xFF || sequence[2] != 0xFF; sequence += 3)
      {
        if (!write_masked(port, sequence[0], sequence[1], sequence[2])) { return false; }
      }
      return true;
    }

    struct register_backup_t
    {
      std::uint8_t reg = 0;
      std::uint8_t value = 0;
      std::uint8_t mask = 0;
    };

    struct register_spec_t
    {
      std::uint8_t reg;
      std::uint8_t mask;
    };

    bool save_registers(int port, const register_spec_t* specs,
                        register_backup_t* saved, std::size_t count)
    {
      for (std::size_t i = 0; i < count; ++i)
      {
        auto value = lgfx::i2c::readRegister8(port, pmic_addr, specs[i].reg, i2c_freq);
        if (!value.has_value()) { return false; }
        saved[i].reg = specs[i].reg;
        saved[i].value = value.value();
        saved[i].mask = specs[i].mask;
      }
      return true;
    }

    bool restore_registers(int port, const register_backup_t* saved, std::size_t count)
    {
      bool restored_all = true;
      while (count)
      {
        --count;
        bool restored = false;
        for (int attempt = 0; attempt < 4; ++attempt)
        {
          if (lgfx::i2c::writeRegister8(port, pmic_addr, saved[count].reg,
                                       saved[count].value & saved[count].mask,
                                       static_cast<std::uint8_t>(~saved[count].mask),
                                       i2c_freq).has_value())
          {
            restored = true;
            break;
          }
        }
        if (!restored)
        {
          restored_all = false;
          ESP_LOGW("board_detect_m5", "PMIC register restore failed reg=%02x",
                   static_cast<unsigned>(saved[count].reg));
        }
      }
      return restored_all;
    }

    enum class panel_variant_t : std::uint8_t { unknown, c, e };

    void write8(soft_spi_t& bus, int pin_cs, std::uint8_t cmd, std::uint8_t data)
    {
      bus.beginTransaction();
      lgfx::gpio_lo(pin_cs);
      bus.writeCommand(cmd, 8);
      bus.writeData(data, 8);
      // An uninitialised Panel_LCD still has _nop_closing=true. Its endWrite()
      // emits NOP before raising CS, so reproduce that probe transaction exactly.
      bus.writeCommand(0x00, 8);
      bus.wait();
      lgfx::gpio_hi(pin_cs);
      bus.endTransaction();
    }

    std::uint8_t read_parameter(soft_spi_t& bus, int pin_cs, std::uint8_t cmd, std::uint8_t index)
    {
      write8(bus, pin_cs, 0xD9, 0x10 | index);
      bus.beginTransaction();
      lgfx::gpio_lo(pin_cs);
      bus.writeCommand(cmd, 8);
      bus.beginRead(1);
      const auto value = static_cast<std::uint8_t>(bus.readData(8));
      lgfx::gpio_hi(pin_cs);
      bus.endRead();
      // Panel_LCD::readCommand() raises CS before endWrite(). The latter still
      // clocks its closing NOP (with CS high) because the panel is uninitialised.
      bus.writeCommand(0x00, 8);
      bus.wait();
      bus.endTransaction();
      return (value >> 1) & 0x7F;
    }

    panel_variant_t probe_panel_variant(soft_spi_t& bus, int pin_cs,
                                        std::uint32_t keys[4], bool try_c_key)
    {
      write8(bus, pin_cs, 0xD9, 0x00);
      write8(bus, pin_cs, 0xDD, 0x01);
      write8(bus, pin_cs, 0xCB, 0x1C);
      keys[0] = read_parameter(bus, pin_cs, 0xDD, 1);
      keys[1] = read_parameter(bus, pin_cs, 0xCB, 1);
      write8(bus, pin_cs, 0xD9, 0x00);
      if (keys[0] == 0x01 && keys[1] == 0x1C) { return panel_variant_t::e; }
      if (!try_c_key) { return panel_variant_t::unknown; }

      bus.beginTransaction();
      lgfx::gpio_lo(pin_cs);
      bus.writeCommand(0xC8, 8);
      bus.writeData(0xFF, 8);
      bus.writeData(0x93, 8);
      bus.writeData(0x42, 8);
      bus.writeCommand(0x00, 8);
      bus.wait();
      lgfx::gpio_hi(pin_cs);
      bus.endTransaction();
      keys[2] = read_parameter(bus, pin_cs, 0xD3, 2);
      keys[3] = read_parameter(bus, pin_cs, 0xD3, 3);
      write8(bus, pin_cs, 0xD9, 0x00);
      return (keys[2] == (0x93 & 0x7F) && keys[3] == 0x42)
           ? panel_variant_t::c : panel_variant_t::unknown;
    }

    panel_variant_t identify_panel_variant(soft_spi_t& bus, int pin_cs,
                                           std::uint32_t keys[4], std::uint32_t poll_ms,
                                           bool try_c_key = true)
    {
      auto variant = probe_panel_variant(bus, pin_cs, keys, try_c_key);
      const auto started = lgfx::millis();
      while (variant == panel_variant_t::unknown && poll_ms)
      {
        lgfx::delay(1);
        variant = probe_panel_variant(bus, pin_cs, keys, try_c_key);
        if (lgfx::millis() - started >= poll_ms) { break; }
      }
      return variant;
    }

    bool is_core_family(const board_result_t& result)
    {
      if (result.def == nullptr) { return false; }
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      return board == lgfx::board_M5StackCore2 || board == lgfx::board_M5Tough;
    }

    bool power_display(board_result_t& result, int port)
    {
      if (result.prepared & prepared_power) { return true; }
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      if (board == lgfx::board_M5Paper)
      {
        pin_level(GPIO_NUM_2, true);
        result.prepared |= prepared_power;
        return true;
      }
      if (!is_core_family(result))
      {
        result.prepared |= prepared_power;
        return true;
      }

      auto chip_id = lgfx::i2c::readRegister8(port, pmic_addr, 0x03, i2c_freq);
      if (!chip_id.has_value() || (chip_id.value() != 0x03 && chip_id.value() != 0x4A)) { return false; }
      const bool is_axp2101 = chip_id.value() == 0x4A;
      const auto enable = lgfx::i2c::readRegister8(port, pmic_addr,
                                                   is_axp2101 ? 0x90 : 0x12, i2c_freq);
      if (!enable.has_value()) { return false; }
      const auto reset_state = is_axp2101
                             ? enable
                             : lgfx::i2c::readRegister8(port, pmic_addr, 0x96, i2c_freq);
      if (!reset_state.has_value()) { return false; }
      const bool power_was_off = !(enable.value() & (is_axp2101 ? 0x08 : 0x04));
      const bool reset_was_low = !(reset_state.value() & 0x02);

      // Only display-related rails and GPIO functions are changed. In
      // particular ALDO3 and DLDO1 retain their application-owned state.
      static constexpr std::uint8_t power192[] = {
        0x95, 0x84, 0x72, 0x28, 0xF0, 0x0F, 0x12, 0x04, 0xFF,
        0x92, 0x00, 0xF8, 0x96, 0x02, 0xFF, 0x94, 0x02, 0xFF,
        0xFF, 0xFF, 0xFF,
      };
      static constexpr std::uint8_t power2101[] = {
        0x90, 0x08, 0xF7, 0x80, 0x05, 0xFF,
        0x82, 0x12, 0x00, 0x84, 0x6A, 0x00,
        0x90, 0x02, 0xFD,
        0xFF, 0xFF, 0xFF,
      };
      if (!write_sequence(port, is_axp2101 ? power2101 : power192)) { return false; }
      if (power_was_off)
      {
        // Pulls can be supplied externally while the card itself is off. Make
        // the powered card pass through CMD58/CMD0 before any LCD traffic.
        result.prepared &= ~prepared_sd_spi;
      }
      lgfx::delay(power_was_off || reset_was_low ? 20 : 5);
      result.prepared |= prepared_power;
      return true;
    }

    bool sd_to_spi(board_result_t& result)
    {
      if (result.prepared & prepared_sd_spi) { return true; }
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      int sclk = -1;
      int miso = -1;
      int mosi = -1;
      int lcd_cs = -1;
      if (board == lgfx::board_M5StackCore2 || board == lgfx::board_M5Tough)
      {
        sclk = GPIO_NUM_18; miso = GPIO_NUM_38; mosi = GPIO_NUM_23;
        lcd_cs = GPIO_NUM_5;
      }
      else if (board == lgfx::board_M5Stack)
      {
        sclk = GPIO_NUM_18; miso = GPIO_NUM_19; mosi = GPIO_NUM_23;
        lcd_cs = GPIO_NUM_14;
      }
      else if (board == lgfx::board_M5Paper)
      {
        sclk = GPIO_NUM_14; miso = GPIO_NUM_13; mosi = GPIO_NUM_12;
        lcd_cs = GPIO_NUM_15;
      }
      else
      {
        return true;
      }

      // The final CS-hold step remains after reset, but SD mode entry is the
      // first shared-wire communication and must not select the panel. This is
      // also required by the complete direct-init path, which has no confirm
      // guard to retain the LCD CS beforehand.
      lgfx::gpio::pin_backup_t lcd_cs_backup(lcd_cs);
      pin_level(lcd_cs, true);
      set_sd_spi_mode_soft(sclk, mosi, miso, GPIO_NUM_4);
      result.prepared |= prepared_sd_spi;
      return true;
    }

    bool stack_reset_and_sample_ips(bool* ips)
    {
      pin_level(GPIO_NUM_33, true);
      lgfx::delay(1);
      lgfx::gpio_lo(GPIO_NUM_33);
      lgfx::delay(2);
      lgfx::pinMode(GPIO_NUM_33, lgfx::pin_mode_t::input_pulldown);
      lgfx::gpio_hi(GPIO_NUM_33);
      *ips = lgfx::gpio_in(GPIO_NUM_33);
      lgfx::pinMode(GPIO_NUM_33, lgfx::pin_mode_t::output);
      lgfx::delay(10);
      return true;
    }

    bool reset_display(board_result_t& result, int port, bool allow_reset,
                       bool* stack_ips = nullptr)
    {
      if (result.prepared & prepared_reset) { return true; }
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      if (board == lgfx::board_M5Station && !allow_reset)
      {
        pin_reset(GPIO_NUM_15, false);
        return true;
      }
      if (board != lgfx::board_M5Paper && !allow_reset) { return true; }

      bool ok = true;
      if (is_core_family(result))
      {
        static constexpr std::uint8_t reset192[] = {
          0x96, 0x00, 0xFD, 0x94, 0x00, 0xFD, 0xFF, 0xFF, 0xFF,
        };
        static constexpr std::uint8_t release192[] = {
          0x96, 0x02, 0xFF, 0x94, 0x02, 0xFF, 0xFF, 0xFF, 0xFF,
        };
        static constexpr std::uint8_t reset2101[] = {
          0x90, 0x00, 0xFD, 0xFF, 0xFF, 0xFF,
        };
        static constexpr std::uint8_t release2101[] = {
          0x90, 0x02, 0xFF, 0xFF, 0xFF, 0xFF,
        };
        const auto chip_id = lgfx::i2c::readRegister8(port, pmic_addr, 0x03, i2c_freq);
        if (!chip_id.has_value() || (chip_id.value() != 0x03 && chip_id.value() != 0x4A))
        {
          return false;
        }
        const bool newer = chip_id.value() == 0x4A;
        ok = write_sequence(port, newer ? reset2101 : reset192);
        if (ok)
        {
          lgfx::delay(1);
          ok = write_sequence(port, newer ? release2101 : release192);
          if (ok) { lgfx::delay(10); }
        }
      }
      else if (board == lgfx::board_M5Stack)
      {
        bool sampled = false;
        ok = stack_reset_and_sample_ips(&sampled);
        if (stack_ips != nullptr) { *stack_ips = sampled; }
      }
      else if (board == lgfx::board_M5Paper)
      {
        pin_reset(GPIO_NUM_23, true);
      }
      else if (board == lgfx::board_M5Station)
      {
        pin_reset(GPIO_NUM_15, true);
      }
      if (ok) { result.prepared |= prepared_reset; }
      return ok;
    }

    void hold_chip_selects(const board_result_t& result)
    {
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      if (board == lgfx::board_M5Station)
      {
        pin_level(GPIO_NUM_5, true);
      }
      else if (board == lgfx::board_M5StackCore2 || board == lgfx::board_M5Tough)
      {
        pin_level(GPIO_NUM_4, true);
        pin_level(GPIO_NUM_5, true);
      }
      else if (board == lgfx::board_M5Stack)
      {
        pin_level(GPIO_NUM_4, true);
        pin_level(GPIO_NUM_14, true);
      }
      else if (board == lgfx::board_M5Paper)
      {
        pin_level(GPIO_NUM_4, true);
        pin_level(GPIO_NUM_15, true);
      }
    }

    bool uses_i2c1(const board_result_t& result)
    {
      const auto board = static_cast<lgfx::board_t>(result.def->id);
      return board == lgfx::board_M5Station
          || board == lgfx::board_M5StackCore2
          || board == lgfx::board_M5Tough;
    }
  }

  const board_def_t* find_board_def(board_id_t board)
  {
    static const board_def_t* const definitions[] = {
      &board_station, &board_core2, &board_tough, &board_stack, &board_paper, nullptr
    };
    for (auto def = definitions; *def != nullptr; ++def)
    {
      if ((*def)->id == board) { return *def; }
    }
    ESP_LOGW("board_detect_m5", "board=%u is not available on the new detection path",
             static_cast<unsigned>(board));
    return nullptr;
  }

  bool prepare(board_result_t& result, const prepare_ctx_t& ctx)
  {
    if (result.def == nullptr || result.def->id == board_id_unknown) { return false; }

    if (!(result.prepared & prepared_power))
    {
      if (detail::is_core_family(result))
      {
        detail::i2c_scope_t i2c(ctx.i2c_port_probe, GPIO_NUM_21, GPIO_NUM_22);
        if (!i2c.opened || !detail::power_display(result, i2c.port)) { return false; }
      }
      else if (!detail::power_display(result, ctx.i2c_port_probe)) { return false; }
    }
    // Detection and preparation leave the hardware SPI host untouched. The
    // final display construction selects its host after these steps complete.
    if (!detail::sd_to_spi(result)) { return false; }
    const bool reset_was_prepared = result.prepared & prepared_reset;
    if (!reset_was_prepared)
    {
      if (detail::is_core_family(result))
      {
        detail::i2c_scope_t i2c(ctx.i2c_port_probe, GPIO_NUM_21, GPIO_NUM_22);
        if (!i2c.opened || !detail::reset_display(result, i2c.port, ctx.allow_reset)) { return false; }
      }
      else if (!detail::reset_display(result, ctx.i2c_port_probe, ctx.allow_reset)) { return false; }
    }
    if (!reset_was_prepared
     && (result.prepared & prepared_reset)
     && detail::is_core_family(result))
    {
      // reset_display() already waits 10 ms. A direct complete specification
      // has no confirm-time polling, so wait the remaining reload upper bound.
      lgfx::delay(110);
    }
    detail::hold_chip_selects(result);
    if (detail::uses_i2c1(result)
     && !lgfx::i2c::init(I2C_NUM_1, GPIO_NUM_21, GPIO_NUM_22).has_value())
    {
      ESP_LOGW("board_detect_m5", "I2C1 could not be opened for SDA=21 SCL=22");
    }
    return true;
  }

  class axp_family_detector_t final : public board_detector_t
  {
  public:
    axp_family_detector_t() : board_detector_t(members_) {}

    bool signature(probe_ctx_t& ctx) const override
    {
      return probe_i2c_ack(ctx, GPIO_NUM_21, GPIO_NUM_22, detail::pmic_addr);
    }

    bool confirm(probe_ctx_t& ctx, board_result_t* result) const override
    {
      detail::i2c_scope_t i2c(ctx, GPIO_NUM_21, GPIO_NUM_22);
      if (!i2c.opened) { return false; }
      auto chip_id = lgfx::i2c::readRegister8(i2c.port, detail::pmic_addr, 0x03, detail::i2c_freq);
      if (!chip_id.has_value()) { return false; }
      const bool is_axp192 = chip_id.value() == 0x03;
      const bool is_axp2101 = chip_id.value() == 0x4A;
      if (!is_axp192 && !is_axp2101) { return false; }
      ESP_LOGD("board_detect_m5", "power controller id=%02x", chip_id.value());

      detail::retained_pin_guard_t lcd_cs(GPIO_NUM_5);
      detail::retained_pin_guard_t sd_cs(GPIO_NUM_4);
      // Even pull probing toggles shared clocks, so deselect the LCD first.
      lcd_cs.activate_high();
      const auto sd_pulls = probe_pin_pulls(detail::sd_pull_mask);
      const bool sd_present = sd_pulls.pulldown_high == detail::sd_pull_mask
                           && sd_pulls.pullup_high == detail::sd_pull_mask;
      std::uint32_t preprepared = 0;
      if (sd_present)
      {
        // This exceptional pre-power transition protects the Station probe on
        // powered Core2 revisions. Unpowered cards transition after PMIC power.
        sd_cs.activate_high();
        detail::set_sd_spi_mode_soft(GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_38, GPIO_NUM_4);
        preprepared |= prepared_sd_spi;
      }

      auto try_station = [&]() -> bool
      {
        if (!is_axp192) { return false; }
        *result = {};
        result->def = &board_station;
        result->prepared = preprepared;
        lgfx::gpio::pin_backup_t reset_backup(GPIO_NUM_15);
        if (!detail::reset_display(*result, i2c.port, ctx.allow_reset))
        {
          reset_backup.restore();
          return false;
        }
        const auto station_id = soft_spi_read32(ctx, GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_23,
                                                GPIO_NUM_19, GPIO_NUM_5, 0x04, 1);
        if ((station_id & 0xFB) != 0x81)
        {
          reset_backup.restore();
          return false;
        }
        // A matching Station keeps reset and LCD CS inactive through prepare.
        lcd_cs.retain();
        return true;
      };

      const bool core_first = ctx.hint == id(lgfx::board_M5StackCore2)
                           || ctx.hint == id(lgfx::board_M5Tough);
      if (!core_first && try_station()) { return true; }

      static constexpr detail::register_spec_t regs192[] = {
        { 0x12, 0x04 }, { 0x28, 0xF0 }, { 0x92, 0x07 },
        { 0x95, 0x8D }, { 0x96, 0x02 }, { 0x94, 0x02 },
      };
      static constexpr detail::register_spec_t regs2101[] = {
        { 0x80, 0x05 }, { 0x82, 0xFF }, { 0x84, 0xFF }, { 0x90, 0x0A },
      };
      detail::register_backup_t saved[6];
      const auto* regs = is_axp192 ? regs192 : regs2101;
      const std::size_t reg_count = is_axp192
                                  ? sizeof(regs192) / sizeof(regs192[0])
                                  : sizeof(regs2101) / sizeof(regs2101[0]);
      if (!detail::save_registers(i2c.port, regs, saved, reg_count))
      {
        return core_first && try_station();
      }

      *result = {};
      result->def = &board_core2;
      result->option = is_axp2101 ? option_core2_new_pmic : 0;
      result->prepared = preprepared;
      lgfx::gpio::pin_backup_t signals[] = {
        GPIO_NUM_15, GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_38
      };
      auto restore_and_fail = [&]() -> bool
      {
        for (auto& pin : signals) { pin.restore(); }
        detail::restore_registers(i2c.port, saved, reg_count);
        return core_first && try_station();
      };
      if (!detail::power_display(*result, i2c.port)) { return restore_and_fail(); }
      sd_cs.activate_high();
      if (!detail::sd_to_spi(*result)) { return restore_and_fail(); }
      detail::hold_chip_selects(*result);

      soft_spi_t bus(GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_23, GPIO_NUM_15);
      bus.init();
      auto panel_id = detail::read_panel_id(bus, GPIO_NUM_5);
      bool reset_before_identify = false;
      if ((panel_id & 0xFF) != 0xE3 && ctx.allow_reset)
      {
        // Some valid panels do not answer until reset. Retry here because a
        // later detection attempt must not relax the caller's reset policy.
        if (!detail::reset_display(*result, i2c.port, true))
        {
          return restore_and_fail();
        }
        reset_before_identify = true;
        panel_id = detail::read_panel_id(bus, GPIO_NUM_5);
      }
      if ((panel_id & 0xFF) != 0xE3)
      {
        return restore_and_fail();
      }

      std::uint32_t keys[4] = {};
      auto variant = detail::identify_panel_variant(
        bus, GPIO_NUM_5, keys, reset_before_identify ? 120 : 1);
      result->prepared |= panel_dirty;
      if (ctx.allow_reset && !reset_before_identify)
      {
        if (!detail::reset_display(*result, i2c.port, true))
        {
          return restore_and_fail();
        }
        std::uint32_t after_keys[4] = {};
        const auto after = detail::identify_panel_variant(
          bus, GPIO_NUM_5, after_keys, 120, variant != detail::panel_variant_t::e);
        if (after != detail::panel_variant_t::unknown)
        {
          variant = after;
          for (int i = 0; i < 4; ++i) { keys[i] = after_keys[i]; }
        }
      }
      if (variant == detail::panel_variant_t::e)
      {
        ESP_LOGI("board_detect_m5", "ILI9342 read-back DDh:%02x CBh:%02x -> ILI9342E",
                 static_cast<unsigned>(keys[0]), static_cast<unsigned>(keys[1]));
      }
      else if (variant == detail::panel_variant_t::c)
      {
        ESP_LOGI("board_detect_m5", "ILI9342 read-back DDh:%02x CBh:%02x ID4:%02x%02x -> ILI9342C",
                 static_cast<unsigned>(keys[0]), static_cast<unsigned>(keys[1]),
                 static_cast<unsigned>(keys[2]), static_cast<unsigned>(keys[3]));
      }
      else
      {
        ESP_LOGW("board_detect_m5", "ILI9342 read-back DDh:%02x CBh:%02x ID4:%02x%02x -> neither key answered, ILI9342C assumed",
                 static_cast<unsigned>(keys[0]), static_cast<unsigned>(keys[1]),
                 static_cast<unsigned>(keys[2]), static_cast<unsigned>(keys[3]));
      }

      const bool tough = lgfx::i2c::readRegister8(i2c.port, 0x2E, 0, detail::i2c_freq).has_value();
      result->def = tough ? &board_tough : &board_core2;
      result->option = (is_axp2101 ? option_core2_new_pmic : 0)
                     | (variant == detail::panel_variant_t::e ? option_lcd_e : 0);
      if (tough) { result->option &= ~option_core2_new_pmic; }
      for (auto& pin : signals) { pin.restore(); }
      sd_cs.retain();
      lcd_cs.retain();
      return true;
    }

  private:
    static const board_def_t* const members_[];
  };

  const board_def_t* const axp_family_detector_t::members_[] = {
    &board_station, &board_core2, &board_tough, nullptr
  };

  class stack_family_detector_t final : public board_detector_t
  {
  public:
    stack_family_detector_t() : board_detector_t(members_) {}

    bool signature(probe_ctx_t& ctx) const override
    {
      auto& values = ctx.detector_workspace.values;
      {
        detail::retained_pin_guard_t lcd_cs(GPIO_NUM_14);
        lcd_cs.activate_high();
        const auto pulls = probe_pin_pulls(detail::sd_pull_mask);
        values[0] = pulls.pulldown_high;
        values[1] = pulls.pullup_high;
      }
      const bool pull_match = values[0] == detail::sd_pull_mask
                           && values[1] == detail::sd_pull_mask;
      const bool bypassed = !pull_match && (ctx.final_attempt || ctx.hint == board_stack.id);
      values[2] = (pull_match ? 1u : 0u) | (bypassed ? 2u : 0u);
      if (!pull_match)
      {
        ESP_LOGD("board_detect_m5",
                 "M5Stack pull signature mismatch pd=%08x%08x pu=%08x%08x",
                 static_cast<unsigned>(values[0] >> 32),
                 static_cast<unsigned>(values[0]),
                 static_cast<unsigned>(values[1] >> 32),
                 static_cast<unsigned>(values[1]));
      }
      return pull_match || bypassed;
    }

    bool confirm(probe_ctx_t& ctx, board_result_t* result) const override
    {
      *result = {};
      result->def = &board_stack;
      lgfx::gpio::pin_backup_t signals[] = {
        GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_27
      };
      detail::retained_pin_guard_t sd_cs(GPIO_NUM_4);
      detail::retained_pin_guard_t lcd_cs(GPIO_NUM_14);
      detail::retained_pin_guard_t reset(GPIO_NUM_33);
      // Both devices are deselected before the first shared-wire operation.
      sd_cs.activate_high();
      lcd_cs.activate_high();
      if (!detail::power_display(*result, ctx.i2c_port_probe)
       || !detail::sd_to_spi(*result))
      {
        for (auto& pin : signals) { pin.restore(); }
        return false;
      }
      reset.activate_high();
      bool ips = false;
      if (!detail::reset_display(*result, ctx.i2c_port_probe, ctx.allow_reset, &ips))
      {
        for (auto& pin : signals) { pin.restore(); }
        return false;
      }
      if (ctx.allow_reset && ips) { result->option |= option_stack_ips; }
      detail::hold_chip_selects(*result);
      const auto panel_id = soft_spi_read32(ctx, GPIO_NUM_18, GPIO_NUM_23, GPIO_NUM_23,
                                            GPIO_NUM_27, GPIO_NUM_14, 0x04, 1);
      if ((panel_id & 0xFF) != 0xE3)
      {
        for (auto& pin : signals) { pin.restore(); }
        return false;
      }
      for (auto& pin : signals) { pin.restore(); }
      sd_cs.retain();
      lcd_cs.retain();
      reset.retain();
      const auto& values = ctx.detector_workspace.values;
      if (values[2] & 2u)
      {
        ESP_LOGI("board_detect_m5",
                 "M5Stack detected after bypassing pull signature pd=%08x%08x pu=%08x%08x",
                 static_cast<unsigned>(values[0] >> 32),
                 static_cast<unsigned>(values[0]),
                 static_cast<unsigned>(values[1] >> 32),
                 static_cast<unsigned>(values[1]));
      }
      return true;
    }

  private:
    static const board_def_t* const members_[];
  };

  const board_def_t* const stack_family_detector_t::members_[] = { &board_stack, nullptr };

  class paper_family_detector_t final : public board_detector_t
  {
  public:
    paper_family_detector_t() : board_detector_t(members_) {}

    bool signature(probe_ctx_t& ctx) const override
    {
      if (ctx.detector_workspace.active) { restore_reset(ctx); }
      static_assert(sizeof(lgfx::gpio::pin_backup_t)
                    <= sizeof(ctx.detector_workspace.object),
                    "detector workspace is too small for pin backup");
      static_assert(alignof(lgfx::gpio::pin_backup_t) <= alignof(std::uint64_t),
                    "detector workspace alignment is insufficient");
      new (ctx.detector_workspace.object) lgfx::gpio::pin_backup_t(GPIO_NUM_23);
      ctx.detector_workspace.active = true;
      lgfx::gpio::pin_backup_t busy(GPIO_NUM_27);
      // This family contract keeps the mandatory reset from stage 1 through
      // stage 2, where prepared_reset records that it already completed.
      detail::pin_reset(GPIO_NUM_23, true);
      lgfx::pinMode(GPIO_NUM_27, lgfx::pin_mode_t::input_pullup);
      const bool matched = !lgfx::gpio_in(GPIO_NUM_27);
      busy.restore();
      if (!matched) { restore_reset(ctx); }
      return matched;
    }

    bool confirm(probe_ctx_t& ctx, board_result_t* result) const override
    {
      if (!ctx.detector_workspace.active) { return false; }
      *result = {};
      result->def = &board_paper;
      result->prepared = prepared_reset;
      lgfx::gpio::pin_backup_t pins[] = {
        GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_12, GPIO_NUM_13,
        GPIO_NUM_14, GPIO_NUM_15, GPIO_NUM_27
      };
      if (!detail::power_display(*result, ctx.i2c_port_probe)
       || !detail::sd_to_spi(*result))
      {
        for (auto& pin : pins) { pin.restore(); }
        restore_reset(ctx);
        return false;
      }
      detail::hold_chip_selects(*result);
      lgfx::pinMode(GPIO_NUM_27, lgfx::pin_mode_t::input);

      soft_spi_t bus(GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, -1);
      bus.init();
      bool matched = false;
      auto started = lgfx::millis();
      while (!lgfx::gpio_in(GPIO_NUM_27) && lgfx::millis() - started <= 1024) { lgfx::delay(1); }
      if (lgfx::gpio_in(GPIO_NUM_27))
      {
        bus.beginTransaction();
        lgfx::gpio_lo(GPIO_NUM_15);
        bus.writeData(__builtin_bswap16(0x6000), 16);
        bus.writeData(__builtin_bswap16(0x0302), 16);
        bus.wait();
        lgfx::gpio_hi(GPIO_NUM_15);
        started = lgfx::millis();
        while (!lgfx::gpio_in(GPIO_NUM_27) && lgfx::millis() - started <= 192) { lgfx::delay(1); }
        lgfx::gpio_lo(GPIO_NUM_15);
        bus.writeData(__builtin_bswap16(0x1000), 16);
        bus.writeData(__builtin_bswap16(0x0000), 16);
        std::uint8_t data[40] = {};
        bus.beginRead();
        bus.readBytes(data, sizeof(data));
        bus.endRead();
        bus.endTransaction();
        lgfx::gpio_hi(GPIO_NUM_15);
        const std::uint32_t panel_size = (std::uint32_t(data[0]) << 24)
                                       | (std::uint32_t(data[1]) << 16)
                                       | (std::uint32_t(data[2]) << 8)
                                       | data[3];
        matched = panel_size == 0x03C0021C;
      }
      if (!matched)
      {
        for (auto& pin : pins) { pin.restore(); }
        restore_reset(ctx);
        return false;
      }

      pins[2].restore(); // MOSI
      pins[3].restore(); // MISO
      pins[4].restore(); // SCLK
      pins[6].restore(); // busy
      release_reset(ctx); // success retains GPIO23 high
      return true;
    }

  private:
    static lgfx::gpio::pin_backup_t* reset_backup(probe_ctx_t& ctx)
    {
      return reinterpret_cast<lgfx::gpio::pin_backup_t*>(ctx.detector_workspace.object);
    }

    static void release_reset(probe_ctx_t& ctx)
    {
      reset_backup(ctx)->~pin_backup_t();
      ctx.detector_workspace.active = false;
    }

    static void restore_reset(probe_ctx_t& ctx)
    {
      reset_backup(ctx)->restore();
      release_reset(ctx);
    }

    static const board_def_t* const members_[];
  };

  const board_def_t* const paper_family_detector_t::members_[] = { &board_paper, nullptr };

  static const axp_family_detector_t axp_family_detector;
  static const stack_family_detector_t stack_family_detector;
  static const paper_family_detector_t paper_family_detector;

  static const board_detector_t* const esp32_d0wdq6_detectors[] = {
    &axp_family_detector,
    &stack_family_detector,
    &paper_family_detector,
    nullptr,
  };

  board_result_t detect_board_family(board_id_t board, probe_ctx_t& ctx)
  {
    for (auto detector = esp32_d0wdq6_detectors; *detector != nullptr; ++detector)
    {
      if (!(*detector)->has_member(board)) { continue; }
      const board_detector_t* const family[] = { *detector, nullptr };
      return detect_board(family, board, ctx);
    }
    board_result_t result;
    return result;
  }
}
}
}
