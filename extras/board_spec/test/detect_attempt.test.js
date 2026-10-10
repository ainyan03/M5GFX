import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { promises as fs } from "node:fs";
import os from "node:os";
import path from "node:path";
import test from "node:test";
import { fileURLToPath } from "node:url";
const here = path.dirname(fileURLToPath(import.meta.url));
const src = path.resolve(here, "../../../src");
import { body, compileRun } from "./detect_host_helpers.js";

test("production attempt yields before construction and preserves adoption and rollback semantics", async () => {
  const main = await fs.readFile(path.join(src, "M5GFX.cpp"), "utf8");
  const attempt = body(main, "static board_detect::detect_outcome_t run_detection_attempt(");
  const compat = body(main, "board_t M5GFX::autodetect(");
  const projection = main.slice(main.indexOf("    const auto board = outcome.setup_succeeded"), main.indexOf("#if defined ( ARDUINO_M5STACK_ATOM )", main.indexOf("    const auto board = outcome.setup_succeeded")));
  const header = await fs.readFile(path.join(src, "M5GFX.h"), "utf8");
  const candidateGetter = body(header, "board_t getBoardCandidate(");
  await compileRun(`
#include "board_detect/detect_session.hpp"
#include <cassert>
#include <initializer_list>
#define CONFIG_IDF_TARGET_ESP32C6 1
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
using namespace m5gfx;
enum class board_t : std::uint32_t {board_unknown=0,member=1};
int rollback_count, commit_count, construct_count, adopt_count, reset_count;
bool power_failed, prepare_ok, adopt_ok, construct_ok, no_display;
namespace lgfx { namespace i2c { bool isInitialized(int) { return false; } } }
namespace m5gfx { namespace board_detect {
struct board_desc_t { board_def_t def; struct { int hw_port; } internal_i2c; struct { const int* data; int size; } hold_high_pins; };
void board_result_t::assign(const board_desc_t* d) { desc=d; def=&d->def; }
struct board_detector_t {};
struct prepare_ctx_t { bool allow_reset; board_id_t preferred; unsigned attempt; bool final_attempt; int i2c_port_probe; detection_transaction_t* transaction; const board_id_t* enabled_ids; };
struct probe_ctx_t : prepare_ctx_t { bool confirm_attempted; };
inline int pins(int) {return 0;} inline int no_pins() {return 0;}
struct detection_transaction_t {
 detection_transaction_t(int,int,bool) {} bool valid() {return true;}
 void rollback() {++rollback_count;} void commit() {++commit_count;}
 void restore_start(const int*, int) {++reset_count;}
 struct buses_t {void opened_i2c(int) {}} buses_;
 buses_t& buses() {return buses_;}
};
board_result_t detected;
board_result_t detect_board(const board_detector_t* const*, board_id_t, probe_ctx_t& p) {p.confirm_attempted=true; return detected;}
bool prepare(const board_desc_t&, board_result_t& r, const prepare_ctx_t&) {if(power_failed) {r.provisional=true;} return prepare_ok;}
namespace m5 {
namespace wiring { namespace detection { const int unconditional_pins=0; } }
struct display_parts_t {int* bus=nullptr;int* panel=nullptr;int* light=nullptr;int* touch=nullptr;};
enum class construct_status_t {failed, no_display, success};
construct_status_t setup_detected_board(const board_result_t&, display_parts_t*) {++construct_count; return !construct_ok ? construct_status_t::failed : no_display ? construct_status_t::no_display : construct_status_t::success;}
void destroy_display_parts(display_parts_t*) {}
struct log_t {const char* name; const char* annotation;};
log_t success_log(const board_result_t&) {return {nullptr,nullptr};}
}
} }
const int probe_i2c_port=0;
template<class SetupDetected> board_detect::detect_outcome_t run_detection_attempt(const board_detect::board_detector_t* const* detectors, const board_detect::detect_request_t& request, bool final_attempt, SetupDetected setup) ${attempt}
struct package_t {const board_detect::board_detector_t* const* detectors;bool conditional_pins_unavailable=false;};
package_t active_package;
package_t select_detection_package(bool) {return active_package;}
bool reject_detected_setup(board_t) {return false;}
struct holder_t {int* get() {return nullptr;}};
struct M5GFX {
 board_t _board=board_t::board_unknown,_board_candidate=board_t::board_unknown;
 board_t getBoard() const {return _board;}
 board_t getBoardCandidate() const ${candidateGetter}
 void project(const board_detect::detect_outcome_t& outcome) { ${projection} }
 struct {board_t fallback_board=board_t::board_unknown;} _detect_config;
 holder_t _panel_last;
 void panel(int*) {}
 bool _adopt_detected_parts(int*,int*,int*,int*) {return true;}
 board_t autodetect(bool use_reset,board_t board,bool final_attempt,bool* transient_fallback,bool* no_signature,board_t* candidate_board) ${compat}
};
int main() {
 using namespace board_detect;
 const board_desc_t desc={{1,"member",0},{-1},{nullptr,0}};
 const board_detector_t d; const board_detector_t* list[]={&d,nullptr};
 for(int provisional=0; provisional<2; ++provisional)
 for(int preferred=0; preferred<3; ++preferred)
 for(int failure=0; failure<4; ++failure)
 for(int displayless=0; displayless<2; ++displayless) {
  rollback_count=commit_count=construct_count=adopt_count=reset_count=0;
  detected={}; detected.assign(&desc); detected.status=detect_status_t::matched;
  power_failed=provisional; prepare_ok=failure!=1; construct_ok=failure!=2; adopt_ok=failure!=3; no_display=displayless;
  detect_request_t req; req.preferred=preferred; req.max_attempts=5;
  auto out=run_detection_session(req,[&](const detect_request_t& r,bool final) {
   return run_detection_attempt(list,r,final,[](m5::display_parts_t&,board_t) {++adopt_count; return adopt_ok;});
  });
  const bool yields=provisional && preferred==2 && prepare_ok;
  const bool success=failure==0 && !yields;
  M5GFX standalone;standalone.project(out);
  assert(standalone.getBoard()==(success?board_t::member:board_t::board_unknown));
  assert(standalone.getBoardCandidate()==(provisional&&!success?board_t::member:board_t::board_unknown));
  assert(out.setup_succeeded==success);
  assert(out.verdict==(provisional ? verdict_t::candidate : verdict_t::confirmed));
  assert(out.candidate_kind==(provisional ? candidate_kind_t::provisional : candidate_kind_t::none));
  assert(out.attempts==(success||yields ? 1:5));
  assert(construct_count==(failure==1||yields ? 0:out.attempts));
  assert(adopt_count==(failure==1||failure==2||yields ? 0:out.attempts));
  assert(commit_count==int(success)); assert(rollback_count==(success ? 0:out.attempts));
  assert(!yields || out.result.candidate==&desc.def);
  assert(should_persist_detection(out,0)==(success&&!provisional));
 }
 // The protected compatibility wrapper must deliver QFN40 weak candidates.
 detected={};detected.candidate=&desc.def;detected.status=detect_status_t::no_match;
 active_package.detectors=list;M5GFX gfx;board_t candidate=board_t::board_unknown;
 bool transient=false,no_signature=false;
 assert(gfx.autodetect(false,board_t::board_unknown,true,&transient,&no_signature,&candidate)==board_t::board_unknown);
 assert(candidate==board_t::member);assert(!transient);
}
`, "attempt");
});

test("public config stores the next preference and warns after either successful or failed init", async () => {
 const main=await fs.readFile(path.join(src,"M5GFX.cpp"),"utf8");
 const header=await fs.readFile(path.join(src,"M5GFX.h"),"utf8");
 const setter=body(main,"void M5GFX::setDetectConfig(");
 const config=body(header,"struct detect_config_t");
 const getter=body(header,"const detect_config_t& getDetectConfig(");
 await compileRun(`
#include <cassert>
#define ESP_PLATFORM 1
int warnings=0;
#define ESP_LOGW(...) (++warnings)
enum class board_t {board_unknown,member,other};
struct detect_config_t ${config};
struct M5GFX {detect_config_t _detect_config;bool _detect_started=false;board_t adopted=board_t::board_unknown;
 void setDetectConfig(const detect_config_t& config) ${setter}
 const detect_config_t& getDetectConfig() const ${getter}
};
int main(){M5GFX gfx;detect_config_t cfg;assert(gfx.getDetectConfig().fallback_board==board_t::board_unknown);
 cfg.fallback_board=board_t::member;gfx.setDetectConfig(cfg);assert(warnings==0);
 gfx._detect_started=true;gfx.setDetectConfig(cfg);assert(warnings==1);assert(gfx.adopted==board_t::board_unknown);
 gfx.adopted=board_t::member;cfg.fallback_board=board_t::other;gfx.setDetectConfig(cfg);
 assert(warnings==2);assert(gfx.adopted==board_t::member);assert(gfx.getDetectConfig().fallback_board==board_t::other);
 cfg.fallback_board=board_t::board_unknown;gfx.setDetectConfig(cfg);assert(gfx.getDetectConfig().fallback_board==board_t::board_unknown);
}
`,"public config");
});
