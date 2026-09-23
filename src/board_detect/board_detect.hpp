// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#pragma once

#include <cstddef>
#include <cstdint>

namespace m5gfx
{
namespace board_detect
{
  using board_id_t = std::uint16_t;
  static constexpr board_id_t board_id_unknown = 0;

  enum board_def_flag_t : std::uint8_t
  {
    // A board with no positive signature. Such a board must be the final member
    // of the final detector; the framework preserves the caller's ordering.
    def_flag_fallback = 1 << 0,
  };

  struct board_def_t
  {
    board_id_t id;
    const char* name;
    std::uint8_t flags;
  };

#if __cplusplus >= 201703L
  inline const board_def_t board_def_unknown = { board_id_unknown, "unknown", 0 };
#else
  // Arduino-ESP32 2.x still compiles as C++11. Internal linkage provides the
  // same ODR safety there; C++17 and newer use the single inline definition.
  static const board_def_t board_def_unknown = { board_id_unknown, "unknown", 0 };
#endif

  enum prepared_state_t : std::uint32_t
  {
    // Bits 0..15 are manufacturer-independent completed operations or probe
    // side effects. Bits 16..31 are reserved for board-specific state.
    prepared_power  = 1u << 0,
    prepared_reset  = 1u << 1,
    prepared_sd_spi = 1u << 2,
    panel_dirty     = 1u << 3,
  };

  enum class detect_status_t : std::uint8_t
  {
    no_match,
    matched,
    excluded,
  };

  struct board_result_t
  {
    const board_def_t* def = &board_def_unknown;
    std::uint32_t option = 0;
    std::uint32_t prepared = 0;
    detect_status_t status = detect_status_t::no_match;
  };

  struct prepare_ctx_t
  {
    bool allow_reset = true;
    int i2c_port_probe = -1;
  };

  struct i2c_scan_cache_t
  {
    std::int16_t pin_sda = -1;
    std::int16_t pin_scl = -1;
    bool pullup_ok = false;
    // Only addresses requested by a detector are probed. `checked` separates
    // a cached NACK from an address that has not been touched yet.
    std::uint32_t checked[4] = {};
    std::uint32_t ack[4] = {};
  };

  struct pin_pull_result_t
  {
    std::uint64_t pulldown_high = 0;
    std::uint64_t pullup_high = 0;
  };

  struct detector_workspace_t
  {
    // Opaque per-detection storage for state carried from signature() to its
    // immediately following confirm(). Only one detector family owns it at a
    // time; a family must release any live object before confirm() returns.
    alignas(std::uint64_t) std::uint8_t object[64] = {};
    std::uint64_t values[4] = {};
    bool active = false;
  };

  struct probe_ctx_t
  {
    // This is the caller's reset policy. A retry must not turn it on: callers
    // that preserve a displayed image depend on every attempt honoring it.
    bool allow_reset = true;
    // The hint moves its detector family forward and may also change the order
    // within that family. It never removes candidates, and a family containing
    // only fallback definitions is never moved forward.
    board_id_t hint = board_id_unknown;
    // The caller's last retry may relax exclusions based only on negative
    // evidence, retaining the legacy broad probe as a final safety net.
    bool final_attempt = false;
    int i2c_port_probe = -1;
    i2c_scan_cache_t i2c_cache;
    detector_workspace_t detector_workspace;
    const board_id_t* enabled_ids = nullptr;
  };

  class board_detector_t
  {
  public:
    constexpr board_detector_t(const board_def_t* const* members_) : members { members_ } {}
    // Stage 1 is a non-destructive family signature and normally restores all
    // state. A family may document a stricter internal contract; for example,
    // Paper pulses and retains reset through its immediately following confirm.
    virtual bool signature(probe_ctx_t& ctx) const = 0;
    virtual bool confirm(probe_ctx_t& ctx, board_result_t* result) const = 0;
    bool has_member(board_id_t id) const;

    // A failed confirmation restores every touched pin and bus. A successful
    // confirmation may retain power-enable, chip-select and reset pins at safe
    // levels until display construction takes ownership; restoring those pins
    // would power a confirmed device down, let another shared-bus device
    // select, or leave the confirmed display's reset input floating. An SD card
    // moved to SPI mode is never moved back to native mode. A caller that
    // rejects a successful result without constructing it cannot assume those
    // retained states are rolled back; cleanup for that case is not provided.
    // The same limitation applies if prepare() fails after confirmation.
    // PMIC-register restoration after a failed confirmation is best effort:
    // failures are warned and detection continues, since aborting would make
    // the transport failure appear to the caller as a different board.
    const board_def_t* const* members;
  };

  board_result_t detect_board(const board_detector_t* const* list, board_id_t hint, probe_ctx_t& ctx);

  bool probe_i2c_ack(probe_ctx_t& ctx, int pin_sda, int pin_scl, std::uint8_t addr);

  // Do not include pins that another device may drive (for example MISO), or
  // pins without internal pulls. A PMIC-switched pull-up may indicate whether
  // its rail is powered, but must not be used as a board signature. U is high
  // in both masks, D in neither, F only in pullup_high, and X only in
  // pulldown_high. Every call measures the requested pins again.
  pin_pull_result_t probe_pin_pulls(std::uint64_t pin_mask);

  // Mode-0 software SPI used only while detecting and preparing a board. Data
  // is laid out like Bus_SPI: low byte first, MSB first within each byte. On a
  // 3-wire ILI9342 bus the panel drives SDA only during the SCLK-low phase and
  // releases it at the rising edge; the board pull-up then restores high in
  // about 0.3 us. Reads therefore sample at the end of the low phase, just
  // before raising SCLK. endTransaction() always restores the shared data line
  // to write direction so a following command can reach the panel.
  class soft_spi_t
  {
  public:
    soft_spi_t(int pin_sclk, int pin_mosi, int pin_miso, int pin_dc,
               std::uint32_t half_us = 1)
    : pin_sclk_(pin_sclk), pin_mosi_(pin_mosi), pin_miso_(pin_miso), pin_dc_(pin_dc),
      half_us_(half_us) {}

    void init();
    void beginTransaction();
    void endTransaction();
    void wait() {}
    void writeCommand(std::uint32_t data, std::uint_fast8_t bits);
    void writeData(std::uint32_t data, std::uint_fast8_t bits);
    std::uint32_t transferData(std::uint32_t data, std::uint_fast8_t bits);
    void beginRead(std::uint_fast8_t dummy_bits = 0);
    std::uint32_t readData(std::uint_fast8_t bits);
    void readBytes(std::uint8_t* dst, std::size_t length);
    void endRead();

  private:
    void clock();
    void send(std::uint32_t data, std::uint_fast8_t bits);
    static std::uint_fast8_t bit_index(std::uint_fast8_t index)
    {
      return (index & ~std::uint_fast8_t(7)) + 7 - (index & 7);
    }

    int pin_sclk_;
    int pin_mosi_;
    int pin_miso_;
    int pin_dc_;
    std::uint32_t half_us_;
  };

  // The first received byte occupies bits 0..7. Bits within each byte arrive MSB first.
  std::uint32_t soft_spi_read32(probe_ctx_t& ctx, int pin_sclk, int pin_mosi, int pin_miso,
                                int pin_dc, int pin_cs, std::uint8_t cmd, std::uint8_t dummy_bits);
}
}
