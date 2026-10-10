import assert from "node:assert/strict";
import { promises as fs } from "node:fs";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";
import { body, compileRun } from "./detect_host_helpers.js";
const src = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../../src");

test("fixed acceptance bypasses NVS and detection and retries clean startup after failure", async () => {
  const main = await fs.readFile(path.join(src, "M5GFX.cpp"), "utf8");
  const registry = await fs.readFile(path.join(src, "board_detect/m5/board_registry.inl"), "utf8");
  const fixed = body(main, "static board_detect::detect_outcome_t run_fixed_detection(");
  const select = body(main, "const board_detect::m5::board_entry_t* select_fixed_board(");
  const finish = body(main, "static board_detect::detect_outcome_t finish_detection_setup(");
  const find = body(registry, "const board_entry_t* find_board(");
  const start = main.indexOf("    _detect_started = true;", main.indexOf("bool M5GFX::init_impl("));
  const prefix = main.slice(start, main.indexOf("    static constexpr char NVS_KEY", start));
  const getterSource = await fs.readFile(path.join(src, "M5GFX.h"), "utf8");
  const fixedGetter = body(getterSource, "board_t getFixedBoard(");
  await compileRun(`
#include "board_detect/detect_session.hpp"
#include <cassert>
#include <cstddef>
#include <cstddef>
#include <initializer_list>
#define ESP_LOGW(...) (++warnings)
#define ESP_LOGI(...) ((void)0)
using namespace m5gfx;
enum class board_t:std::uint32_t {board_unknown=0,member=1,unsupported=2};
int warnings,nvs_calls,detect_calls,construct_calls,adopt_calls,finish_calls,rollback_calls,commit_calls,start_calls;
bool prepare_ok,construct_ok,adopt_ok,displayless;
namespace lgfx {namespace i2c {bool isInitialized(int) {return false;}}}
namespace m5gfx {namespace board_detect {
constexpr int max_detection_pins=64;
struct list_t {const std::int8_t* data;std::size_t size;};
using pin_list_t=list_t;
struct board_desc_t {
 board_def_t def;
 struct {int hold_pin;} power;
 struct {int pin;} reset;
 struct {int sclk,mosi,miso,dc,cs,rst,busy;} display;
 struct {int sclk,mosi,miso,sd_cs,other_cs;} sd;
 struct {int sda,scl,hw_port;} internal_i2c;
 list_t hold_high_pins,op_gpio_pins;
};
void board_result_t::assign(const board_desc_t* value) {desc=value;def=value?&value->def:&board_def_unknown;}
struct prepare_ctx_t {bool allow_reset=true,collect_reset_option=false;int i2c_port_probe=-1;detection_transaction_t* transaction=nullptr;};
inline pin_list_t no_pins() {return {nullptr,0};}
struct detection_transaction_t {
 detection_transaction_t(pin_list_t pins,pin_list_t,bool) {
  for(std::size_t i=0;i<pins.size;++i) {assert(pins.data[i]>=0&&pins.data[i]<=7);}
 }
 bool valid() {return true;}
 void rollback() {++rollback_calls;}void commit(){++commit_calls;}
 void restore_start(const std::int8_t*,std::size_t) {}
 struct buses_t {void opened_i2c(int){}} buses_;
 buses_t& buses(){return buses_;}
};
namespace startup_detail {bool description_valid(const board_desc_t&) {return true;}}
bool prepare(const board_desc_t&,board_result_t& result,const prepare_ctx_t& ctx) {
 assert(ctx.collect_reset_option);
 assert(result.option==0&&result.prepared==0&&!result.provisional&&result.refine==nullptr&&result.candidate==nullptr);
 ++start_calls;result.option=4;result.prepared=prepared_power;return prepare_ok;
}
namespace m5 {
struct display_parts_t {int* bus=nullptr;int* panel=nullptr;int* light=nullptr;int* touch=nullptr;};
enum class construct_status_t {ok,no_display,failed};
using start_t=bool(*)(board_result_t&,const prepare_ctx_t&);
struct board_entry_t {const board_desc_t* desc;start_t fixed_start;};
template<std::size_t BoardCount> const board_entry_t* find_board(const board_entry_t (&boards)[BoardCount],board_id_t id) ${find}
const board_desc_t desc={{1,"member",0},{0},{1},{2,3,4,5,6,1,7},{-1,-1,-1,-1,-1},{-1,-1,-1},{nullptr,0},{nullptr,0}};
const board_entry_t entries[]={{&desc,nullptr}};
const board_entry_t (&esp32_d0wdq6_boards)[1]=entries;
construct_status_t setup_detected_board(const board_result_t& result,display_parts_t*) {
 assert(result.desc==&desc&&result.def==&desc.def&&result.option==4&&result.prepared==prepared_power&&!result.provisional&&result.refine==nullptr);
 ++construct_calls;return !construct_ok?construct_status_t::failed:displayless?construct_status_t::no_display:construct_status_t::ok;
}
void destroy_display_parts(display_parts_t*) {}
struct log_t {const char* name;const char* annotation;};
log_t success_log(const board_result_t&){return {nullptr,nullptr};}
}
}}
const int probe_i2c_port=-1;
template<class SetupDetected> board_detect::detect_outcome_t finish_detection_setup(board_detect::board_result_t& result,board_detect::detect_outcome_t outcome,board_detect::detection_transaction_t& transaction,board_t setup_board,SetupDetected setup) ${finish}
template<class SetupDetected> board_detect::detect_outcome_t run_fixed_detection(const board_detect::m5::board_entry_t& entry,bool allow_reset,SetupDetected setup) ${fixed}
const board_detect::m5::board_entry_t* select_fixed_board(board_t board) ${select}
bool reject_detected_setup(board_t){return false;}
struct M5GFX {
 bool _detect_started=false;
 board_t _board=board_t::board_unknown,_board_candidate=board_t::member,_fixed_board=board_t::board_unknown;
 struct {board_t fixed_board=board_t::board_unknown,fallback_board=board_t::board_unknown;} _detect_config;
 board_t getBoard() const {return _board;}
 board_t getFixedBoard() const ${fixedGetter}
 bool _adopt_detected_parts(int*,int*,int*,int*){++adopt_calls;return adopt_ok;}
 bool _finish_detected_init(bool){++finish_calls;return true;}
 bool init(bool use_reset=true,bool use_clear=true) {
 ${prefix}
 ++nvs_calls;++detect_calls;return false;
 }
};
int main() {
 using namespace board_detect;
 prepare_ok=construct_ok=adopt_ok=true;
 M5GFX unsupported;unsupported._detect_config.fixed_board=board_t::unsupported;
 assert(!unsupported.init()&&unsupported.getFixedBoard()==board_t::board_unknown);
 assert(nvs_calls==0&&detect_calls==0&&start_calls==0);
 for(int failure=0;failure<3;++failure) {
 M5GFX gfx;gfx._detect_config.fixed_board=board_t::member;
 prepare_ok=failure!=0;construct_ok=failure!=1;adopt_ok=failure!=2;
 assert(!gfx.init());assert(gfx.getBoard()==board_t::board_unknown&&gfx.getFixedBoard()==board_t::member);
 prepare_ok=construct_ok=adopt_ok=true;
 assert(gfx.init());assert(gfx.getBoard()==board_t::member&&gfx.getFixedBoard()==board_t::member);
 assert(gfx._board_candidate==board_t::board_unknown);
 assert(nvs_calls==0&&detect_calls==0);
 }
 assert(commit_calls==3&&rollback_calls==3);
 M5GFX wide;wide._detect_config.fixed_board=static_cast<board_t>(0x10001);
 assert(!wide.init()&&wide.getFixedBoard()==board_t::board_unknown);
 displayless=true;M5GFX no_display;no_display._detect_config.fixed_board=board_t::member;
 assert(no_display.init()&&no_display.getBoard()==board_t::member&&no_display.getFixedBoard()==board_t::member);
 M5GFX initialized;initialized._board=board_t::member;initialized._detect_config.fixed_board=board_t::unsupported;
 const int before=start_calls;assert(initialized.init()&&initialized.getFixedBoard()==board_t::board_unknown&&start_calls==before);
 assert(nvs_calls==0&&detect_calls==0);
 M5GFX auto_gfx;assert(!auto_gfx.init());assert(nvs_calls==1&&detect_calls==1);
}
`, "fixed entry and startup");
});
