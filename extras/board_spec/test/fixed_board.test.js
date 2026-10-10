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

test("fixed variant callbacks never invoke family identification", async () => {
  const detector = await fs.readFile(path.join(src, "board_detect/board_detect.inl"), "utf8");
  const core = await fs.readFile(path.join(src, "board_detect/m5/esp32_d0wdq6.inl"), "utf8");
  const s3 = await fs.readFile(path.join(src, "board_detect/m5/esp32s3/cores3.inl"), "utf8");
  const families = await fs.readFile(path.join(src, "board_detect/m5/esp32s3/families.inl"), "utf8");
  const coreStart = body(core, "bool fixed_start_core(board_result_t& result, const prepare_ctx_t& ctx)\n    {");
  assert.doesNotMatch(coreStart, /touch_|try_station|probe_pin_pulls|assign\(/);
  const s3Start = body(s3, "bool fixed_start(board_result_t& result");
  assert.doesNotMatch(s3Start, /camera_id|probe_dedicated|firmware_reg|assign\(/);
  for (const name of ["atoms3", "atoms3r", "airq"]) {
    const start = body(families, `bool fixed_start_${name}(`);
    assert.match(start, /fixed_start_spi_variant/);
    assert.doesNotMatch(start, /camera|signature|detector|refine/);
  }
  const spiStart = body(detector, "bool fixed_start_spi_variant(");
  assert.doesNotMatch(spiStart, /detector|candidate|assign\(/);
});

test("fixed Core2 and CoreS3 observe only construction variants in startup order", async () => {
  const core = await fs.readFile(path.join(src,"board_detect/m5/esp32_d0wdq6.inl"),"utf8");
  const s3 = await fs.readFile(path.join(src,"board_detect/m5/esp32s3/cores3.inl"),"utf8");
  const coreStart=body(core,"bool fixed_start_core(board_result_t& result, const prepare_ctx_t& ctx)\n    {");
  const s3Start=body(s3,"bool fixed_start(board_result_t& result");
  await compileRun(`
#include "board_detect/detect_types.hpp"
#include <cassert>
#include <cstddef>
#include <vector>
#define ESP_LOGW(...) (++warnings)
using namespace m5gfx::board_detect;
std::vector<int> sequence;bool pmic_known, new_pmic, lcd_e, powered;int warnings;
namespace m5gfx {namespace board_detect {
struct board_desc_t {board_def_t def;struct {int sda,scl;} internal_i2c;struct {std::int8_t dc,sclk,mosi,miso;} display;};
void board_result_t::assign(const board_desc_t* d){desc=d;def=&d->def;}
struct detection_transaction_t {void restore_start(const std::int8_t(&)[4]){}};
struct prepare_ctx_t {detection_transaction_t* transaction;int i2c_port_probe;};
struct probe_ctx_t:prepare_ctx_t {};
struct pmic_variant_t {std::uint32_t detected_option;};
}}
namespace generated_options {namespace core2 {constexpr unsigned new_pmic=1,lcd_e=2;}}
constexpr unsigned vbus_5v=4;
enum class panel_variant_t {unknown,c,e};
namespace startup_detail {
struct i2c_scope_t {bool opened=true;int port=0;i2c_scope_t(detection_transaction_t&,int,const decltype(board_desc_t::internal_i2c)&){}};
bool prepare_power(const board_desc_t&,board_result_t& r,int,bool) {sequence.push_back(2);r.prepared|=prepared_power;return powered;}
}
const pmic_variant_t* observe_core_pmic(int){sequence.push_back(1);static pmic_variant_t p;p.detected_option=new_pmic?1:0;return pmic_known?&p:nullptr;}
bool observe_core_panel(board_result_t&,const prepare_ctx_t&,panel_variant_t& v,std::uint32_t[4]){sequence.push_back(3);v=lcd_e?panel_variant_t::e:panel_variant_t::unknown;return true;}
void log_panel_variant(panel_variant_t,const std::uint32_t[4]){}
bool prepare(const board_desc_t&,board_result_t&,const prepare_ctx_t&){sequence.push_back(4);return true;}
std::uint32_t observe_vbus(probe_ctx_t&){sequence.push_back(1);return vbus_5v;}
void enable_bus_out(const prepare_ctx_t&){sequence.push_back(3);}
bool refine_panel(board_result_t& r,const prepare_ctx_t&){sequence.push_back(5);if(lcd_e)r.option|=2;return true;}
bool fixed_core(board_result_t& result,const prepare_ctx_t& ctx) ${coreStart}
bool fixed_s3(board_result_t& result,const prepare_ctx_t& ctx) ${s3Start}
int main(){
 const board_desc_t desc={{1,"fixed",0},{0,1},{2,3,4,5}};detection_transaction_t tx;prepare_ctx_t ctx={&tx,-1};
 board_result_t r;r.assign(&desc);pmic_known=false;powered=true;
 assert(!fixed_core(r,ctx));assert(sequence==std::vector<int>({1}));assert(warnings==1);
 for(bool pmic:{false,true})for(bool panel:{false,true}){
  sequence.clear();r={};r.assign(&desc);pmic_known=true;new_pmic=pmic;lcd_e=panel;
  assert(fixed_core(r,ctx));assert(sequence==std::vector<int>({1,2,3,4}));
  assert(r.option==unsigned((pmic?1:0)|(panel?2:0)));assert(r.def==&desc.def&&!r.provisional&&r.refine==nullptr);
 }
 sequence.clear();r={};r.assign(&desc);lcd_e=true;
 assert(fixed_s3(r,ctx));assert(sequence==std::vector<int>({1,2,3,5,4}));
 assert(r.option==(vbus_5v|2));assert(r.def==&desc.def&&r.refine==nullptr);
}
`,"fixed construction variants");
});

test("fixed Tough backlight follows observed PMIC while its panel and touch remain fixed", async () => {
 const setup=await fs.readFile(path.join(src,"board_detect/m5/esp32_d0wdq6_setup.inl"),"utf8");
 const construct=body(setup,"construct_status_t construct_tough(");
 await compileRun(`
#include "board_detect/detect_types.hpp"
#include <cassert>
#include <cstddef>
#include <memory>
using namespace m5gfx::board_detect;
int selected;
namespace lgfx {struct ILight {virtual ~ILight()=default;};struct Touch_CHSC6540 {}; }
struct Light_M5StackCore2_AXP2101:lgfx::ILight {Light_M5StackCore2_AXP2101(){selected=1;}};
struct Light_M5Tough:lgfx::ILight {Light_M5Tough(){selected=2;}};
struct panel_t {void touch(lgfx::Touch_CHSC6540*){}};
struct display_parts_t{};
struct display_parts_owner_t {
 std::unique_ptr<lgfx::ILight> light;
 std::unique_ptr<lgfx::Touch_CHSC6540> touch;
 std::unique_ptr<panel_t> panel;
 bool release_to(display_parts_t*){return true;}
};
namespace generated_options {namespace core2 {constexpr unsigned new_pmic=1;}namespace tough {constexpr unsigned lcd_e=2;}}
constexpr int bus_tough=0,touch_tough=0;
template<class T>T* make_default_part(){return new T();}
template<class T>T* make_i2c_touch(int){return new T();}
void construct_core_panel(const board_result_t&,unsigned,int,display_parts_owner_t* out){out->panel.reset(new panel_t);}
using construct_status_t=bool;bool construct_status(bool v){return v;}
construct_status_t construct_tough(const board_result_t& result,display_parts_t* parts) ${construct}
int main(){for(unsigned option=0;option<2;++option){board_result_t r;r.option=option;display_parts_t p;assert(construct_tough(r,&p));assert(selected==(option?1:2));}}
`,"Tough PMIC backlight");
});

test("Paper IT8951 initialization waits for BUSY before sending panel commands", async () => {
 const panel=await fs.readFile(path.join(src,"lgfx/v1/panel/Panel_IT8951.inl"),"utf8");
 const init=body(panel,"bool Panel_IT8951::init(");
 assert.ok(init.indexOf("_wait_busy();")>=0&&init.indexOf("_wait_busy();")<init.indexOf("startWrite();"));
});

test("fixed SPI variants read only the selected display and keep default on unreadable ID", async () => {
 const source=await fs.readFile(path.join(src,"board_detect/board_detect.inl"),"utf8");
 const observe=body(source,"bool observe_spi_variant(");
 const start=body(source,"bool fixed_start_spi_variant(");
 await compileRun(`
#include "board_detect/detect_types.hpp"
#include <cassert>
#include <cstddef>
#define ESP_LOGW(...) (++warnings)
using namespace m5gfx::board_detect;
int mode,reads,warnings,prepares;
namespace m5gfx {namespace board_detect {
struct board_desc_t {board_def_t def;struct {std::int8_t sclk,mosi,miso,dc,cs;} display;};
void board_result_t::assign(const board_desc_t* d){desc=d;def=&d->def;}
struct detection_transaction_t {void restore_start(const std::int8_t(&pins)[4]){for(int i=0;i<4;++i)assert(pins[i]>=2&&pins[i]<=5);}};
struct prepare_ctx_t {detection_transaction_t* transaction;};
struct probe_ctx_t:prepare_ctx_t {};
struct spi_id_probe_t {unsigned cmd,dummy_bits,mask;const std::uint32_t* values;unsigned value_count,option_bit;};
struct spi_id_member_t {const board_desc_t* desc;const spi_id_probe_t* probes;unsigned probe_count;bool three_wire;std::uint8_t slow_retry_half_us;bool legacy_zero_preamble;};
}}
bool prepare(const board_desc_t&,board_result_t& r,const prepare_ctx_t&){++prepares;r.prepared=prepared_power|prepared_reset;return true;}
std::uint32_t soft_spi_read32(probe_ctx_t&,int sclk,int mosi,int miso,int dc,int cs,unsigned,unsigned,unsigned half,bool){
 assert(sclk==2&&mosi==3&&miso==3&&dc==5&&cs==6);assert(half==1||half==5);++reads;
 return mode==0?0xffffffff:mode==1?0x11:mode==3&&reads==1?0xffffffff:0x22;
}
bool observe_spi_variant(probe_ctx_t& ctx,const board_desc_t& desc,const spi_id_probe_t* probes,std::size_t probe_count,std::uint32_t* option,bool three_wire,std::uint8_t slow_retry_half_us,bool legacy_zero_preamble) ${observe}
bool fixed_start_spi_variant(board_result_t& result,const prepare_ctx_t& ctx,const spi_id_member_t& member) ${start}
int main(){
 const board_desc_t desc={{1,"fixed display",0},{2,3,4,5,6}};
 const std::uint32_t standard[]={0x11},alternate[]={0x22};
 const spi_id_probe_t probes[]={{4,1,0xff,standard,1,0},{4,1,0xff,alternate,1,1}};
 const spi_id_member_t member={&desc,probes,2,true,5,false};detection_transaction_t tx;prepare_ctx_t ctx={&tx};
 for(mode=0;mode<4;++mode){board_result_t r;r.assign(&desc);reads=warnings=prepares=0;
 assert(fixed_start_spi_variant(r,ctx,member));assert(r.def==&desc.def&&r.option==unsigned(mode>=2));
 assert(prepares==1&&reads==(mode==0||mode==3?2:1)&&warnings==(mode==0));
 assert(r.prepared==(prepared_power|prepared_reset)&&!r.provisional&&r.refine==nullptr);
 }
}
`,"fixed SPI-only observation");
});
