// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#pragma once

#include "board_detect.hpp"

#include <esp_log.h>

namespace m5gfx
{
namespace board_detect
{
  bool board_detector_t::has_member(board_id_t id) const
  {
    if (members == nullptr) { return false; }
    for (auto p = members; *p != nullptr; ++p)
    {
      if ((*p)->id == id) { return true; }
    }
    return false;
  }

  namespace
  {
    static constexpr char tag[] = "board_detect";

    bool enabled(const probe_ctx_t& ctx, board_id_t id)
    {
      if (ctx.enabled_ids == nullptr) { return true; }
      for (auto p = ctx.enabled_ids; *p != board_id_unknown; ++p)
      {
        if (*p == id) { return true; }
      }
      return false;
    }

    bool fallback_only(const board_detector_t* detector)
    {
      if (detector->members == nullptr || detector->members[0] == nullptr) { return false; }
      for (auto p = detector->members; *p != nullptr; ++p)
      {
        if (!((*p)->flags & def_flag_fallback)) { return false; }
      }
      return true;
    }

    bool run_detector(const board_detector_t* detector, probe_ctx_t& ctx, board_result_t* result)
    {
      const char* family = (detector->members && detector->members[0])
                         ? detector->members[0]->name : "empty";
      const bool signature = detector->signature(ctx);
      ESP_LOGD(tag, "stage=1 family=%s detector=%p match=%d", family,
               static_cast<const void*>(detector), signature);
      if (!signature) { return false; }

      board_result_t candidate;
      const bool confirmed = detector->confirm(ctx, &candidate);
      ESP_LOGD(tag, "stage=2 family=%s detector=%p match=%d board=%u name=%s", family,
               static_cast<const void*>(detector), confirmed,
               static_cast<unsigned>(candidate.def ? candidate.def->id : board_id_unknown),
               candidate.def ? candidate.def->name : "invalid");
      if (!confirmed) { return false; }

      if (candidate.def == nullptr || candidate.def->id == board_id_unknown)
      {
        ESP_LOGW(tag, "detector=%p returned a match without a board definition",
                 static_cast<const void*>(detector));
        return false;
      }
      candidate.status = enabled(ctx, candidate.def->id)
                       ? detect_status_t::matched
                       : detect_status_t::excluded;
      *result = candidate;
      return true;
    }
  }

  board_result_t detect_board(const board_detector_t* const* list, board_id_t hint, probe_ctx_t& ctx)
  {
    board_result_t result;
    if (list == nullptr) { return result; }
    ctx.hint = hint;

    const board_detector_t* hinted = nullptr;
    if (hint != board_id_unknown)
    {
      bool hint_found = false;
      for (auto p = list; *p != nullptr; ++p)
      {
        if ((*p)->has_member(hint))
        {
          hint_found = true;
          if (!fallback_only(*p)) { hinted = *p; }
          else
          {
            ESP_LOGD(tag, "hint=%u belongs to a fallback-only detector",
                     static_cast<unsigned>(hint));
          }
          break;
        }
      }
      if (!hint_found)
      {
        // A hint belonging to a detector outside this package-specific list is normal.
        ESP_LOGD(tag, "hint=%u is not present in detector list", static_cast<unsigned>(hint));
      }
      else if (hinted != nullptr)
      {
        if (run_detector(hinted, ctx, &result)) { return result; }
        ESP_LOGD(tag, "hint=%u did not match its detector family", static_cast<unsigned>(hint));
      }
    }

    for (auto p = list; *p != nullptr; ++p)
    {
      if (*p == hinted) { continue; }
      if (run_detector(*p, ctx, &result)) { return result; }
    }
    return result;
  }

  bool probe_i2c_ack(probe_ctx_t& ctx, int pin_sda, int pin_scl, std::uint8_t addr)
  {
    // Reserved addresses are never touched, even if requested accidentally.
    if (addr < 0x08 || addr > 0x77) { return false; }

    auto& cache = ctx.i2c_cache;
    if (cache.pin_sda != pin_sda || cache.pin_scl != pin_scl)
    {
      cache = {};
      cache.pin_sda = pin_sda;
      cache.pin_scl = pin_scl;

      lgfx::gpio::pin_backup_t backup[] = { pin_sda, pin_scl };
      lgfx::pinMode(pin_sda, lgfx::pin_mode_t::input_pulldown);
      lgfx::pinMode(pin_scl, lgfx::pin_mode_t::input_pulldown);
      lgfx::delayMicroseconds(10);
      lgfx::pinMode(pin_sda, lgfx::pin_mode_t::input);
      lgfx::pinMode(pin_scl, lgfx::pin_mode_t::input);
      lgfx::delayMicroseconds(10);
      cache.pullup_ok = lgfx::gpio_in(pin_sda) && lgfx::gpio_in(pin_scl);
      for (auto& pin : backup) { pin.restore(); }
    }

    const std::uint32_t bit = 1u << (addr & 31);
    if (!(cache.checked[addr >> 5] & bit))
    {
      cache.checked[addr >> 5] |= bit;
      if (cache.pullup_ok)
      {
        lgfx::gpio::pin_backup_t backup[] = { pin_sda, pin_scl };
        if (lgfx::i2c::init(ctx.i2c_port_probe, pin_sda, pin_scl).has_value())
        {
          const bool hit = lgfx::i2c::beginTransaction(ctx.i2c_port_probe, addr, 100000, false).has_value()
                        && lgfx::i2c::endTransaction(ctx.i2c_port_probe).has_value();
          if (hit) { cache.ack[addr >> 5] |= bit; }
          lgfx::i2c::release(ctx.i2c_port_probe);
        }
        for (auto& pin : backup) { pin.restore(); }
      }
    }
    return cache.pullup_ok && (cache.ack[addr >> 5] & bit);
  }

  pin_pull_result_t probe_pin_pulls(std::uint64_t pin_mask)
  {
    static constexpr std::size_t max_pins = 64;
    pin_pull_result_t result;

    for (std::size_t pin = 0; pin < max_pins; ++pin)
    {
      const std::uint64_t bit = std::uint64_t(1) << pin;
      if (!(pin_mask & bit)) { continue; }
      // Measure one pin completely before touching the next. On a native-mode
      // SD bus this avoids raising CLK while CMD is temporarily pulled low.
      lgfx::gpio::pin_backup_t backup(pin);
      lgfx::pinMode(pin, lgfx::pin_mode_t::input_pulldown);
      lgfx::delayMicroseconds(10);
      if (lgfx::gpio_in(pin)) { result.pulldown_high |= bit; }
      lgfx::pinMode(pin, lgfx::pin_mode_t::input_pullup);
      lgfx::delayMicroseconds(10);
      if (lgfx::gpio_in(pin)) { result.pullup_high |= bit; }
      backup.restore();
    }
    return result;
  }

  void soft_spi_t::init()
  {
    lgfx::gpio_lo(pin_sclk_);
    lgfx::pinMode(pin_sclk_, lgfx::pin_mode_t::output);
    lgfx::gpio_hi(pin_mosi_);
    lgfx::pinMode(pin_mosi_, lgfx::pin_mode_t::output);
    if (pin_miso_ != pin_mosi_) { lgfx::pinMode(pin_miso_, lgfx::pin_mode_t::input); }
    if (pin_dc_ >= 0) { lgfx::pinMode(pin_dc_, lgfx::pin_mode_t::output); }
  }

  void soft_spi_t::beginTransaction()
  {
    lgfx::gpio_lo(pin_sclk_);
  }

  void soft_spi_t::endTransaction()
  {
    endRead();
    lgfx::gpio_lo(pin_sclk_);
  }

  void soft_spi_t::clock()
  {
    lgfx::delayMicroseconds(half_us_);
    lgfx::gpio_hi(pin_sclk_);
    lgfx::delayMicroseconds(half_us_);
    lgfx::gpio_lo(pin_sclk_);
    lgfx::delayMicroseconds(half_us_);
  }

  void soft_spi_t::send(std::uint32_t data, std::uint_fast8_t bits)
  {
    for (std::uint_fast8_t i = 0; i < bits; ++i)
    {
      if (data & (std::uint32_t(1) << bit_index(i))) { lgfx::gpio_hi(pin_mosi_); }
      else                                           { lgfx::gpio_lo(pin_mosi_); }
      clock();
    }
  }

  void soft_spi_t::writeCommand(std::uint32_t data, std::uint_fast8_t bits)
  {
    if (pin_dc_ >= 0) { lgfx::gpio_lo(pin_dc_); }
    send(data, bits);
  }

  void soft_spi_t::writeData(std::uint32_t data, std::uint_fast8_t bits)
  {
    if (pin_dc_ >= 0) { lgfx::gpio_hi(pin_dc_); }
    send(data, bits);
  }

  std::uint32_t soft_spi_t::transferData(std::uint32_t data, std::uint_fast8_t bits)
  {
    if (pin_dc_ >= 0) { lgfx::gpio_hi(pin_dc_); }
    std::uint32_t value = 0;
    for (std::uint_fast8_t i = 0; i < bits; ++i)
    {
      const auto index = bit_index(i);
      if (data & (std::uint32_t(1) << index)) { lgfx::gpio_hi(pin_mosi_); }
      else                                    { lgfx::gpio_lo(pin_mosi_); }
      lgfx::delayMicroseconds(half_us_);
      if (lgfx::gpio_in(pin_miso_)) { value |= std::uint32_t(1) << index; }
      lgfx::gpio_hi(pin_sclk_);
      lgfx::delayMicroseconds(half_us_);
      lgfx::gpio_lo(pin_sclk_);
    }
    return value;
  }

  void soft_spi_t::beginRead(std::uint_fast8_t dummy_bits)
  {
    if (pin_dc_ >= 0) { lgfx::gpio_hi(pin_dc_); }
    if (pin_miso_ == pin_mosi_) { lgfx::pinMode(pin_mosi_, lgfx::pin_mode_t::input); }
    for (std::uint_fast8_t i = 0; i < dummy_bits; ++i) { clock(); }
  }

  std::uint32_t soft_spi_t::readData(std::uint_fast8_t bits)
  {
    std::uint32_t value = 0;
    for (std::uint_fast8_t i = 0; i < bits; ++i)
    {
      lgfx::delayMicroseconds(half_us_);
      if (lgfx::gpio_in(pin_miso_)) { value |= std::uint32_t(1) << bit_index(i); }
      lgfx::gpio_hi(pin_sclk_);
      lgfx::delayMicroseconds(half_us_);
      lgfx::gpio_lo(pin_sclk_);
    }
    return value;
  }

  void soft_spi_t::readBytes(std::uint8_t* dst, std::size_t length)
  {
    while (length--) { *dst++ = static_cast<std::uint8_t>(readData(8)); }
  }

  void soft_spi_t::endRead()
  {
    if (pin_miso_ == pin_mosi_) { lgfx::pinMode(pin_mosi_, lgfx::pin_mode_t::output); }
  }

  std::uint32_t soft_spi_read32(probe_ctx_t&, int pin_sclk, int pin_mosi, int pin_miso,
                                int pin_dc, int pin_cs, std::uint8_t cmd, std::uint8_t dummy_bits)
  {
    lgfx::gpio::pin_backup_t backup_sclk(pin_sclk);
    lgfx::gpio::pin_backup_t backup_mosi(pin_mosi);
    lgfx::gpio::pin_backup_t backup_miso(pin_miso);
    lgfx::gpio::pin_backup_t backup_dc(pin_dc);
    lgfx::gpio::pin_backup_t backup_cs(pin_cs);

    lgfx::gpio_hi(pin_cs);
    lgfx::pinMode(pin_cs, lgfx::pin_mode_t::output);
    soft_spi_t bus(pin_sclk, pin_mosi, pin_miso, pin_dc);
    bus.init();
    bus.beginTransaction();
    lgfx::gpio_lo(pin_cs);
    bus.writeCommand(cmd, 8);
    bus.beginRead(dummy_bits);
    const auto value = bus.readData(32);
    lgfx::gpio_hi(pin_cs);
    bus.endTransaction();

    backup_cs.restore();
    backup_dc.restore();
    backup_miso.restore();
    backup_mosi.restore();
    backup_sclk.restore();
    return value;
  }
}
}
